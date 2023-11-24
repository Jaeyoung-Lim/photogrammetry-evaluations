/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim. All rights reserved.
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

#ifndef COLMAP_IO_H
#define COLMAP_IO_H

#include "GeographicLib/Geoid.hpp"
#include "adaptive_viewutility/adaptive_viewutility.h"
#include "photogrammetry_evaluations/geo_conversions.h"

namespace colmapio {

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
      /// TODO: Figure out why this is needed
      local_attitude(2) = -local_attitude(2);
      // local_attitude(1) = -local_attitude(1);
      auto viewpoint = std::make_shared<ViewPoint>(idx++, local_position, local_attitude);
      viewpoint->setImageName(image_name);
      viewpoints.push_back(viewpoint);
    }
  }

  return true;
}

}  // namespace colmapio

#endif
