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

#include "photogrammetry_evaluations/evaluation.h"

#include "adaptive_viewutility/adaptive_viewutility.h"
#include "adaptive_viewutility/evaluation.h"
#include "terrain_navigation/data_logger.h"
#include "terrain_navigation/visualization.h"

#include <pcl/io/obj_io.h>
#include "grid_map_pcl/GridMapPclConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "photogrammetry_evaluations/geo_conversions.h"

#include "GeographicLib/Geoid.hpp"

#include <filesystem>

void ReadViewset(const std::string path, std::vector<PathSegment> &view_set) {
  // Write data to files
  bool parse_result;

  std::ifstream file(path);
  std::string str;

  // Look for the image file name in the path
  while (getline(file, str)) {
    std::stringstream ss(str);
    std::vector<std::string> data;
    std::string cc;
    while (getline(ss, cc, ',')) {
      data.push_back(cc);
    }
    // #   ID, x, y, z, qw, qx, qy, qz
    ss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6] >> data[7];
    if (data[0] == "id") continue;
    State state;
    state.position << std::stof(data[1]), std::stof(data[2]), std::stof(data[3]);
    state.attitude << std::stof(data[4]), std::stof(data[5]), std::stof(data[6]), std::stof(data[7]);

    PathSegment view;
    view.states.push_back(state);
    view_set.push_back(view);
  }
  return;
}

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

bool getViewPointFromImage(std::string &image_path, std::string image_name, std::shared_ptr<ViewPoint> &viewpoint,
                           int idx, Eigen::Vector3d map_origin) {
  GDALDataset *poSrcDS = (GDALDataset *)GDALOpen(image_path.c_str(), GA_ReadOnly);

  if (!poSrcDS) return false;

  auto egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  std::string exif_gps_altitude = poSrcDS->GetMetadataItem("EXIF_GPSAltitude");
  std::string exif_gps_latitude = poSrcDS->GetMetadataItem("EXIF_GPSLatitude");
  std::string exif_gps_longitude = poSrcDS->GetMetadataItem("EXIF_GPSLongitude");
  std::string exif_gps_track = poSrcDS->GetMetadataItem("EXIF_GPSTrack");

  double viewpoint_latitude = StringToGeoReference(exif_gps_latitude);
  double viewpoint_longitude = StringToGeoReference(exif_gps_longitude);
  double viewpoint_altitude =
      StringToGeoReference(exif_gps_altitude) +
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(viewpoint_latitude, viewpoint_longitude);  // AMSL altitude
  std::cout << "latitude: " << viewpoint_latitude << " longitude: " << viewpoint_longitude
            << " altitude: " << viewpoint_altitude << std::endl;

  double time_seconds = GetTimeInSeconds(std::string(poSrcDS->GetMetadataItem("EXIF_DateTime")));

  /// TODO: Set viewpoint local position from geo reference
  Eigen::Vector3d lv03_viewpoint_position;
  GeoConversions::forward(viewpoint_latitude, viewpoint_longitude, viewpoint_altitude, lv03_viewpoint_position.x(),
                          lv03_viewpoint_position.y(), lv03_viewpoint_position.z());

  Eigen::Vector3d local_position = lv03_viewpoint_position - map_origin;

  std::cout << "Local position: " << local_position.transpose() << std::endl;
  Eigen::Vector4d local_attitude{1.0, 0.0, 0.0, 0.0};
  viewpoint = std::make_shared<ViewPoint>(idx, local_position, local_attitude);

  viewpoint->setTime(time_seconds);
  viewpoint->setImage(image_path);
  viewpoint->setImageName(image_name);

  GDALClose((GDALDatasetH)poSrcDS);

  return true;
}

void writePositionsToFile(std::string output_path, std::vector<std::shared_ptr<ViewPoint>> viewpoints) {
  std::shared_ptr<DataLogger> camera_logger = std::make_shared<DataLogger>();
  camera_logger->setKeys({"file", "X", "Y", "Z"});
  camera_logger->setSeparator(" ");

  for (auto &view : viewpoints) {
    auto center_position = view->getCenterLocal();
    std::unordered_map<std::string, std::any> camera_state;
    camera_state.insert(std::pair<std::string, std::string>("file", view->getImageName()));
    camera_state.insert(std::pair<std::string, double>("X", center_position.x()));
    camera_state.insert(std::pair<std::string, double>("Y", center_position.y()));
    camera_state.insert(std::pair<std::string, double>("Z", center_position.z()));
    camera_logger->record(camera_state);
  }
  camera_logger->writeToFile(output_path);
}

bool parsePoseFromText(std::string text_path, std::string image_file, Eigen::Vector3d &position,
                       Eigen::Vector4d &attitude) {
  bool parse_result;

  std::ifstream file(text_path);
  std::string str;

  // Look for the image file name in the path
  while (getline(file, str)) {
    if (str.find(image_file) != std::string::npos) {
      std::stringstream ss(str);
      std::vector<std::string> camera_pose;
      camera_pose.resize(9);
      // #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
      ss >> camera_pose[0] >> camera_pose[1] >> camera_pose[2] >> camera_pose[3] >> camera_pose[4] >> camera_pose[5] >>
          camera_pose[6] >> camera_pose[7] >> camera_pose[8];
      attitude << std::stof(camera_pose[1]), std::stof(camera_pose[2]), std::stof(camera_pose[3]),
          std::stof(camera_pose[4]);
      position << std::stof(camera_pose[5]), std::stof(camera_pose[6]), std::stof(camera_pose[7]);
      return true;
    }
  }
  return false;
}

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

bool getViewPointFromCOLMAP(std::string path, std::vector<std::shared_ptr<ViewPoint>> &reference,
                            std::vector<std::shared_ptr<ViewPoint>> &viewpoints) {
  int idx{0};
  /// TODO: Parse camera views from text file

  std::cout << "Reading camera file: " << path << std::endl;

  for (auto reference_view : reference) {
    std::string image_name = reference_view->getImageName();
    Eigen::Vector3d view_position;
    Eigen::Vector4d view_attitude;
    if (parsePoseFromText(path, image_name, view_position, view_attitude)) {
      auto R = quat2RotMatrix(view_attitude);
      Eigen::Vector3d local_position = -R.transpose() * view_position;
      Eigen::Vector4d view_offset = Eigen::Vector4d(std::cos(M_PI_2), std::sin(M_PI_2), 0.0, 0.0);
      Eigen::Vector4d local_attitude = quatMultiplication(view_offset, view_attitude);
      ///TODO: Figure out why this is needed
      local_attitude(2) = -local_attitude(2);
      // local_attitude(1) = -local_attitude(1);
      auto viewpoint = std::make_shared<ViewPoint>(idx++, local_position, local_attitude);
      viewpoint->setImageName(image_name);
      viewpoints.push_back(viewpoint);
    }
  }

  return true;
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

  std::cout << "Map origin: " << map_origin << std::endl;

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
      if (getViewPointFromImage(image_path, image_name, viewpoint, idx++, map_origin)) {
        viewpoint_list.push_back(viewpoint);
      }
    }
  }

  std::cout << "Viewpoint size: " << viewpoint_list.size() << std::endl;

  publishViewpoints(viewpoint_pub, viewpoint_list, Eigen::Vector3d(0.0, 0.0, 1.0));

  writePositionsToFile(output_dir_path + "/camera.txt", viewpoint_list);

  /// Read colmap aligned camera poses and project over DEM
  /// Visualize dense reconstructed mesh from COLMAP
  std::vector<std::shared_ptr<ViewPoint>> reconstructed_viewpoints;
  getViewPointFromCOLMAP(camera_file, viewpoint_list, reconstructed_viewpoints);
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
