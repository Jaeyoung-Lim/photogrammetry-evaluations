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

struct VehicleState {
  double timestamp{0};
  int image_count{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector4d attitude{Eigen::Vector4d::Zero()};
};

void readTrajectory(const std::string path, std::vector<VehicleState> &vehicle_states,
                    std::vector<Trajectory> &view_set) {
  // Write data to files
  bool parse_result;

  std::ifstream file(path);
  std::string str;

  // Look for the image file name in the path
  int view_count{0};
  while (getline(file, str)) {
    std::stringstream ss(str);
    std::vector<std::string> data;
    std::string cc;
    while (getline(ss, cc, ',')) {
      data.push_back(cc);
    }
    // #  "timestamp", "coverage", "quality", "image_count", "position_x", "position_y", "position_z",
    //    "attitude_w", "attitude_x", "attitude_y", "attitude_z"
    ss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6] >> data[7] >> data[8] >> data[9] >>
        data[10];
    int image_count = std::stoi(data[3]);
    Eigen::Vector3d vehicle_position(std::stof(data[4]), std::stof(data[5]), std::stof(data[6]));
    Eigen::Vector4d vehicle_attitude(std::stof(data[7]), std::stof(data[8]), std::stof(data[9]), std::stof(data[10]));
    VehicleState vehicle_state;
    vehicle_state.timestamp = std::stof(data[0]);
    vehicle_state.image_count = image_count;
    vehicle_state.position = vehicle_position;
    vehicle_state.attitude = vehicle_attitude;
    vehicle_states.push_back(vehicle_state);
    if (image_count > view_count) {
      State state;
      state.position = vehicle_position;
      state.attitude = vehicle_attitude;
      Trajectory view;
      view.states.push_back(state);
      view_set.push_back(view);
      view_count = image_count;
    }
  }
  return;
}

void publishCameraPath(const ros::Publisher pub, std::vector<VehicleState> &vehicle_states) {
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  for (auto viewpoint : vehicle_states) {
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(),
                                     vector3d2PoseStampedMsg(viewpoint.position, viewpoint.attitude));
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

  std::string benchmark_dir_path, viewset_path, trajectory_path, gt_path;
  double replay_time{50.0};
  bool visualization_enabled{true};
  nh_private.param<std::string>("viewset_path", viewset_path, "resources/executed_viewset.csv");
  nh_private.param<std::string>("trajectory_path", trajectory_path, "resources/executed_viewset.csv");
  nh_private.param<std::string>("benchmark_dir_path", benchmark_dir_path, "output");
  nh_private.param<bool>("visualize", visualization_enabled, true);
  nh_private.param<double>("replay_time", replay_time, replay_time);
  nh_private.param<std::string>("groundtruth_mesh_path", gt_path, "");

  grid_map::GridMap gt_map =
      grid_map::GridMap({"roi", "elevation", "elevation_normal_x", "elevation_normal_y", "elevation_normal_z",
                         "visibility", "geometric_prior", "normalized_prior"});
  std::shared_ptr<ViewUtilityMap> groundtruth_map = std::make_shared<ViewUtilityMap>(gt_map);
  double resolution = 1.0;
  groundtruth_map->initializeFromMesh(gt_path, resolution);

  if (benchmark_dir_path.empty()) {
    std::cout << "Missing groundtruth mesh or the estimated mesh" << std::endl;
    return 1;
  }

  std::vector<VehicleState> vehicle_states;
  std::vector<VehicleState> state_history;
  std::vector<Trajectory> candidate_viewpoints;
  if (!trajectory_path.empty()) {
    readTrajectory(trajectory_path, vehicle_states, candidate_viewpoints);
  }

  if (visualization_enabled) {
    int instance{0};
    int increment{10};
    std::vector<ViewPoint> viewpoints;

    ros::Duration(2.0).sleep();

    int view_count{0};
    for (auto &state : vehicle_states) {
      double time = state.timestamp;
      if (time > replay_time) {
        break;
      }
      state_history.push_back(state);

      int image_count = state.image_count;
      if (image_count > view_count) {
        viewpoints.push_back(ViewPoint(instance, candidate_viewpoints[image_count-1].states[0].position,
                                       candidate_viewpoints[image_count-1].states[0].attitude));
        view_count = image_count;
        if (view_count % increment == 0) {
          grid_map::GridMap est_map;

          std::string map_path =
              benchmark_dir_path + "/" + std::to_string(int(view_count / increment)-1) + "/dense/meshed-poisson.ply";
          std::cout << "  - map_path: " << map_path << std::endl;
          // Check if file exists
          if (file_exists(map_path)) {
            std::shared_ptr<ViewUtilityMap> estimated_map = std::make_shared<ViewUtilityMap>(est_map);
            estimated_map->initializeFromMesh(map_path, resolution);

            Evaluation::CompareMapLayer(groundtruth_map->getGridMap(), estimated_map->getGridMap());

            printGridmapInfo("Estimated map", estimated_map->getGridMap());
            MapPublishOnce(est_map_pub, estimated_map);
          }
        }
        instance++;
        publishViewpoint(viewpoint_pub, viewpoints, Eigen::Vector3d(0.0, 0.0, 1.0));
        publishCameraPath(camera_path_pub, state_history);
      }
    }
  }
  std::cout << "Finished replay" << std::endl;
  ros::spin();
  return 0;
}
