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

void compareMapLayers(grid_map::GridMap &map, const std::string reference_layer, const std::string layer) {
  std::vector<double> error_list;
  map.add("error");

  double max_error = -std::numeric_limits<double>::infinity();
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    // Using at position allows us to use different resolution maps
    double error = map.at(reference_layer, index) - map.at(layer, index);
    if (std::isfinite(error)) {
      double abs_error = std::abs(error);
      map.at("error", index) = abs_error;
      if (abs_error > max_error) {
        max_error = abs_error;
      }
      error_list.push_back(abs_error);
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
  std::string dem_path, dem_color_path, mesh_path, output_dir_path, camera_file;
  bool visualization_enabled{true};
  nh_private.param<std::string>("viewset_path", viewset_path, "");
  nh_private.param<std::string>("dem_path", dem_path, "");
  nh_private.param<std::string>("mesh_path", mesh_path, "");
  nh_private.param<std::string>("camera_file", camera_file, "");
  nh_private.param<std::string>("dem_color_path", dem_color_path, "");
  nh_private.param<std::string>("output_dir_path", output_dir_path, "");
  nh_private.param<bool>("visualize", visualization_enabled, true);

  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->Load(dem_path, false, dem_color_path);

  /// TODO: Parse exif tags from the images and visualize above DEM
  ESPG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map->getGlobalOrigin(map_coordinate, map_origin);
  terrain_map->AddLayerNormals("elevation");

  auto utility_map = std::make_shared<ViewUtilityMap>(terrain_map->getGridMap());

  std::cout << "Map origin: " << map_origin.transpose() << std::endl;

  /// Iterate through image files
  std::vector<std::shared_ptr<ViewPoint>> viewpoint_list;
  int idx{0};
  for (const auto &dirEntry : std::filesystem::recursive_directory_iterator(viewset_path)) {
    if (dirEntry.path().extension() == ".JPG") {
      std::string image_path = dirEntry.path().string();
      std::string image_name = dirEntry.path().filename();
      std::cout << "Reading file : " << dirEntry.path().string() << std::endl;
      std::cout << "  - File name: " << image_name << std::endl;
      std::shared_ptr<ViewPoint> viewpoint;
      if (exifio::getViewPointFromImage(image_path, image_name, viewpoint, idx++, map_origin)) {
        viewpoint_list.push_back(viewpoint);
      }
    }
  }

  std::cout << "Viewpoint size: " << viewpoint_list.size() << std::endl;

  publishViewpoints(viewpoint_pub, viewpoint_list, Eigen::Vector3d(0.0, 0.0, 1.0));

  /// Read colmap aligned camera poses and project over DEM
  /// Visualize dense reconstructed mesh from COLMAP
  std::vector<std::shared_ptr<ViewPoint>> reconstructed_viewpoints;
  colmapio::getViewPointFromCOLMAP(camera_file, viewpoint_list, reconstructed_viewpoints);
  std::vector<ViewPoint> dereferenced_viewpoints;
  for (auto view : reconstructed_viewpoints) {
    dereferenced_viewpoints.push_back(*view);
  }
  /// Compare view utility metrics
  double utility = utility_map->CalculateViewUtility(dereferenced_viewpoints, true);

  publishViewpoints(reconstructed_viewpoint_pub, reconstructed_viewpoints, Eigen::Vector3d(1.0, 1.0, 0.0));

  grid_map::GridMap reconstructed_map = terrain_map->getGridMap();
  pcl::PolygonMesh mesh;
  pcl::io::loadPLYFile(mesh_path, mesh);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(mesh, "reconstruction", reconstructed_map);
  grid_map::GridMapPclConverter::addColorLayerFromPolygonMesh(mesh, "reconstruction_color", reconstructed_map);

  /// Compare mesh file with DEM
  compareMapLayers(reconstructed_map, "elevation", "reconstruction");

  /// Publish maps
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), msg);
  terrain_map_pub.publish(msg);

  grid_map_msgs::GridMap mesh_msg;
  grid_map::GridMapRosConverter::toMessage(reconstructed_map, mesh_msg);
  mesh_msg.info.header.frame_id = "map";
  reconstructed_map_pub.publish(mesh_msg);

  ros::spin();
  return 0;
}
