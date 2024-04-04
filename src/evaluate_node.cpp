/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Node to test planner in the view utiltiy map
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "adaptive_viewutility/evaluation.h"
#include "terrain_navigation/data_logger.h"

#include <pcl/io/obj_io.h>
#include "grid_map_pcl/GridMapPclConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "photogrammetry_evaluations/colmap_io.h"
#include "photogrammetry_evaluations/geo_conversions.h"

#include <filesystem>
#include "photogrammetry_evaluations/exif_io.h"

void publishCameraPath(const ros::Publisher pub, const std::vector<ViewPoint> viewpoints) {
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  for (auto viewpoint : viewpoints) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(),
                                     vector3d2PoseStampedMsg(viewpoint.getCenterLocal(), viewpoint.getOrientation()));
  }

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posestampedhistory_vector;

  pub.publish(msg);
}

void publishViewpoints(const ros::Publisher &viewpoint_pub, std::vector<std::shared_ptr<ViewPoint>> viewpoints,
                       const Eigen::Vector3d color) {
  std::vector<visualization_msgs::Marker> viewpoint_vector;

  int i = 0;
  for (std::shared_ptr<ViewPoint> viewpoint : viewpoints) {
    viewpoint_vector.insert(viewpoint_vector.begin(), Viewpoint2MarkerMsg(i, *viewpoint, color));
    i++;
  }

  visualization_msgs::MarkerArray viewpoint_marker_msg;
  viewpoint_marker_msg.markers = viewpoint_vector;
  viewpoint_pub.publish(viewpoint_marker_msg);
}

void compareMapLayers(grid_map::GridMap &map, const std::string roi_layer, const std::string layer, const std::string ground_truth) {
  std::vector<double> error_list;
  map.add("error");

  double max_error = -std::numeric_limits<double>::infinity();
  double error_threshold{10.0};
  int total_count{0};
  int count_roi_cells{0};
  int count_completeness{0};
  int count_precision{0};
  int count_reconstruction{0};
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    // Using at position allows us to use different resolution maps
    total_count++;
    if (map.at(roi_layer, index) > 0.5) { // Inside ROI
      /// Only iterate over that are inside the ROI
      if (std::isfinite(map.at(ground_truth, index))) {
        count_roi_cells++;
        double error = map.at(ground_truth, index) - map.at(layer, index);
        double abs_error = std::abs(error);
        if (abs_error < error_threshold) {
          count_completeness++;
        }  
      }
      if (std::isfinite(map.at(layer, index))) {
        count_reconstruction++;
        double error = map.at(ground_truth, index) - map.at(layer, index);
        double abs_error = std::abs(error);
        if (abs_error < error_threshold) {
          count_precision++;
        }  
      }
      double error = map.at(ground_truth, index) - map.at(layer, index);
      if (std::isfinite(error)) {
        double abs_error = std::abs(error);
        map.at("error", index) = abs_error;
        if (abs_error > max_error) {
          max_error = abs_error;
        }
        error_list.push_back(abs_error);
      } else {
        // std::cout << "No Reconstruction exists!" << std::endl;
      }
    }
  }
  double mean = {0.0};
  for (const auto error : error_list) {
    mean += (error / error_list.size());
  }
  double rmse_squared = {0.0};
  for (const auto error : error_list) {
    rmse_squared += std::pow(error - mean, 2) / error_list.size();
  }
  double rmse = std::sqrt(rmse_squared);
  std::cout << "Completeness: " << double(count_completeness)/double(count_roi_cells)<<std::endl;
  std::cout << "Precision: " << double(count_precision)/double(count_reconstruction)<<std::endl;
  std::cout << "  total count        : " << total_count << std::endl;
  std::cout << "  completeness count : " << count_completeness << std::endl;
  std::cout << "  roi count          : " << count_roi_cells << std::endl;
  std::cout << "mean: " << mean << std::endl;
  std::cout << "RMSE: " << rmse << std::endl;
  std::cout << "Max Error: " << max_error << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher terrain_map_pub = nh.advertise<grid_map_msgs::GridMap>("terrain", 1, true);
  ros::Publisher reconstructed_map_pub = nh.advertise<grid_map_msgs::GridMap>("reconstruction", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);
  ros::Publisher reconstructed_viewpoint_pub =
      nh.advertise<visualization_msgs::MarkerArray>("reconstructed_viewpoints", 1, true);

  std::string viewset_path;
  std::string dem_path, dem_color_path, mesh_path, output_dir_path, camera_file, camera_parameters, gt_mesh_path, roi_path;
  bool visualization_enabled{true};
  nh_private.param<std::string>("viewset_path", viewset_path, "");
  nh_private.param<std::string>("dem_path", dem_path, "");
  nh_private.param<std::string>("mesh_path", mesh_path, "");
  nh_private.param<std::string>("roi_path", roi_path, "");
  nh_private.param<std::string>("gt_mesh_path", gt_mesh_path, "");
  nh_private.param<std::string>("camera_file", camera_file, "");
  nh_private.param<std::string>("camera_parameters", camera_parameters, "");
  nh_private.param<std::string>("dem_color_path", dem_color_path, "");
  nh_private.param<std::string>("output_dir_path", output_dir_path, "");
  nh_private.param<bool>("visualize", visualization_enabled, true);

  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->Load(dem_path, false, dem_color_path);

  auto roi_map = std::make_shared<TerrainMap>();
  roi_map->Load(roi_path, false);
  /// TODO: Parse exif tags from the images and visualize above DEM
  ESPG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map->getGlobalOrigin(map_coordinate, map_origin);
  terrain_map->AddLayerNormals("elevation");
  roi_map->setGlobalOrigin(map_coordinate, map_origin);
  auto utility_map = std::make_shared<ViewUtilityMap>(terrain_map->getGridMap());

  std::cout << "Map origin: " << map_origin.transpose() << std::endl;
  terrain_map->getGridMap().add("roi");
  for (grid_map::GridMapIterator iterator(terrain_map->getGridMap()); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    Eigen::Vector2d position;
    terrain_map->getGridMap().getPosition(index, position);
    if(roi_map->getGridMap().isInside(position) ){
      if (std::isfinite(roi_map->getGridMap().atPosition("elevation", position))) {
        terrain_map->getGridMap().at("roi", index) = 1.0;
      } else {
        terrain_map->getGridMap().at("roi", index) = 0.0;
      }
    } else {
        terrain_map->getGridMap().at("roi", index) = 0.0;
    }
  }


  grid_map::GridMap& reconstructed_map = terrain_map->getGridMap();
  pcl::PolygonMesh mesh;
  pcl::io::loadPLYFile(mesh_path, mesh);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(mesh, "reconstruction", reconstructed_map);
  grid_map::GridMapPclConverter::addColorLayerFromPolygonMesh(mesh, "reconstruction_color", reconstructed_map);

  pcl::PolygonMesh gt_mesh;
  pcl::io::loadPLYFile(gt_mesh_path, gt_mesh);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(gt_mesh, "groundtruth", reconstructed_map);
  grid_map::GridMapPclConverter::addColorLayerFromPolygonMesh(gt_mesh, "groundtruth_color", reconstructed_map);
  /// Compare mesh file with DEM
  compareMapLayers(reconstructed_map, "roi", "reconstruction", "groundtruth");

  /// Publish maps
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), msg);
  terrain_map_pub.publish(msg);

  grid_map_msgs::GridMap mesh_msg;
  grid_map::GridMapRosConverter::toMessage(reconstructed_map, mesh_msg);
  mesh_msg.info.header.frame_id = "map";
  reconstructed_map_pub.publish(mesh_msg);

  /// TODO: Write map information into a csv file
  // writeMapToFile(output_dir_path + "/map.csv", reconstructed_map);

  ros::spin();
  return 0;
}
