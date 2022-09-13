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

void ReadViewset(const std::string path, std::vector<Trajectory> &view_set) {
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

    Trajectory view;
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

void publishViewpoint(const ros::Publisher &viewpoint_pub, std::vector<ViewPoint> viewpoints,
                      const Eigen::Vector3d color) {
  std::vector<visualization_msgs::Marker> viewpoint_vector;

  int i = 0;
  for (auto viewpoint : viewpoints) {
    viewpoint_vector.insert(viewpoint_vector.begin(), Viewpoint2MarkerMsg(i, viewpoint, color));
    i++;
  }

  visualization_msgs::MarkerArray viewpoint_marker_msg;
  viewpoint_marker_msg.markers = viewpoint_vector;
  viewpoint_pub.publish(viewpoint_marker_msg);
}

inline bool file_exists(const std::string &name) {
  std::ifstream f(name.c_str());
  return f.good();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ros::Publisher camera_path_pub = nh.advertise<nav_msgs::Path>("camera_path", 1, true);
  ros::Publisher est_map_pub = nh.advertise<grid_map_msgs::GridMap>("estimated_map", 1, true);
  ros::Publisher viewpoint_pub = nh.advertise<visualization_msgs::MarkerArray>("viewpoints", 1, true);

  std::string benchmark_dir_path, viewset_path;
  int maximum_number_views{50};
  bool visualization_enabled{true};
  nh_private.param<std::string>("viewset_path", viewset_path, "resources/executed_viewset.csv");
  nh_private.param<std::string>("benchmark_dir_path", benchmark_dir_path, "output");
  nh_private.param<bool>("visualize", visualization_enabled, true);
  nh_private.param<int>("maximum_number_views", maximum_number_views, maximum_number_views);

  if (benchmark_dir_path.empty()) {
    std::cout << "Missing groundtruth mesh or the estimated mesh" << std::endl;
    return 1;
  }

  std::vector<Trajectory> candidate_viewpoints;
  if (!viewset_path.empty()) {
    ReadViewset(viewset_path, candidate_viewpoints);
  }

  double resolution = 1.0;

  if (visualization_enabled) {
    int instance{0};
    int increment{5};
    std::vector<ViewPoint> viewpoints;

    ros::Duration(2.0).sleep();

    while (true) {
      if (instance > maximum_number_views) {
          break;
      }
      std::cout << "Publishing map!" << std::endl;
      /// TODO: Load viewpoint progress
      if (instance < candidate_viewpoints.size()) {
        viewpoints.push_back(ViewPoint(instance, candidate_viewpoints[instance].states[0].position,
                                       candidate_viewpoints[instance].states[0].attitude));
      }

      publishViewpoint(viewpoint_pub, viewpoints, Eigen::Vector3d(0.0, 0.0, 1.0));
      publishCameraPath(camera_path_pub, viewpoints);
      if (instance % increment == 0) {
        grid_map::GridMap est_map;

        std::string map_path =
            benchmark_dir_path + "/" + std::to_string(int(instance / increment)) + "/dense/meshed-poisson.ply";
        std::cout << "  - map_path: " << map_path << std::endl;
        // Check if file exists
        std::cout << "instance: " << instance << std::endl;
        if (file_exists(map_path)) {
          std::shared_ptr<ViewUtilityMap> estimated_map = std::make_shared<ViewUtilityMap>(est_map);
          estimated_map->initializeFromMesh(map_path, resolution);
          printGridmapInfo("Estimated map", estimated_map->getGridMap());
          MapPublishOnce(est_map_pub, estimated_map);
        }
      }
      ros::Duration(0.1).sleep();
      instance++;
    }
  }
  ros::spin();
  return 0;
}
