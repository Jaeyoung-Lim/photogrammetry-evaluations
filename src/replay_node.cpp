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
#include "terrain_navigation/visualization.h"

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "terrain_navigation/profiler.h"
#include "photogrammetry_evaluations/geo_conversions.h"

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

bool getViewPointFromImage(std::string &image_path, std::shared_ptr<ViewPoint> &viewpoint, int idx, Eigen::Vector3d map_origin) {
  GDALDataset *poSrcDS = (GDALDataset *)GDALOpen(image_path.c_str(), GA_ReadOnly);

  if (!poSrcDS) return false;

  std::string exif_gps_altitude = poSrcDS->GetMetadataItem("EXIF_GPSAltitude");
  std::string exif_gps_latitude = poSrcDS->GetMetadataItem("EXIF_GPSLatitude");
  std::string exif_gps_longitude = poSrcDS->GetMetadataItem("EXIF_GPSLongitude");
  std::string exif_gps_track = poSrcDS->GetMetadataItem("EXIF_GPSTrack");

  double viewpoint_altitude = StringToGeoReference(exif_gps_altitude);
  double viewpoint_latitude = StringToGeoReference(exif_gps_latitude);
  double viewpoint_longitude = StringToGeoReference(exif_gps_longitude);
  std::cout << "latitude: " << viewpoint_latitude << " longitude: " << viewpoint_longitude
            << " altitude: " << viewpoint_altitude << std::endl;
  
  double time_seconds = GetTimeInSeconds(std::string(poSrcDS->GetMetadataItem("EXIF_DateTime")));
  
  ///TODO: Set viewpoint local position from geo reference
  Eigen::Vector3d lv03_viewpoint_position;
  GeoConversions::forward(viewpoint_latitude, viewpoint_longitude, viewpoint_altitude, lv03_viewpoint_position.x(), lv03_viewpoint_position.y(), lv03_viewpoint_position.z());

  Eigen::Vector3d local_position = lv03_viewpoint_position - map_origin;

  std::cout << "Local position: " << local_position.transpose() << std::endl;
  Eigen::Vector4d local_attitude{1.0, 0.0, 0.0, 0.0};
  viewpoint = std::make_shared<ViewPoint>(idx++, local_position, local_attitude);

  viewpoint->setTime(time_seconds);
  viewpoint->setImage(image_path);

  GDALClose((GDALDatasetH)poSrcDS);

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher terrain_map_pub = nh.advertise<grid_map_msgs::GridMap>("terrain", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

  std::string viewset_path;
  std::string dem_path, dem_color_path;
  bool visualization_enabled{true};
  nh_private.param<std::string>("viewset_path", viewset_path, "");
  nh_private.param<std::string>("dem_path", dem_path, "");
  nh_private.param<std::string>("dem_color_path", dem_color_path, "");
  nh_private.param<bool>("visualize", visualization_enabled, true);

  auto terrain_map = std::make_shared<GridMapGeo>();
  terrain_map->Load(dem_path, false, dem_color_path);

  /// TODO: Parse exif tags from the images and visualize above DEM
  ESPG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map->getGlobalOrigin(map_coordinate, map_origin);

  std::cout << "Map origin: " << map_origin << std::endl;

  /// Iterate through image files
  std::vector<std::shared_ptr<ViewPoint>> viewpoint_list;
  int idx{0};
  for (const auto &dirEntry : std::filesystem::recursive_directory_iterator(viewset_path)) {
    if (dirEntry.path().extension() == ".JPG") {
      std::string image_path = dirEntry.path().string();
      std::cout << "Reading file : " << dirEntry.path().string() << std::endl;
      std::shared_ptr<ViewPoint> viewpoint;
      if (getViewPointFromImage(image_path, viewpoint, idx++, map_origin)) {
        viewpoint_list.push_back(viewpoint);
      }
    }
  }

  std::cout << "Viewpoint size: " << viewpoint_list.size() << std::endl;


  publishViewpoints(viewpoint_pub, viewpoint_list, Eigen::Vector3d(0.0, 0.0, 1.0));

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(terrain_map->getGridMap(), msg);
  terrain_map_pub.publish(msg);

  /// TODO: Align mesh in colmap
  /// TODO: Parse mesh file from COLMAP

  /// TODO: Compare mesh file with DEM

  ros::spin();
  return 0;
}
