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
#include "photogrammetry_evaluations/colmap_io.h"
#include "photogrammetry_evaluations/exif_io.h"
#include "photogrammetry_evaluations/geo_conversions.h"

#include "GeographicLib/Geoid.hpp"

#include <filesystem>

int main(int argc, char **argv) {
  ros::init(argc, argv, "adaptive_viewutility");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  std::string viewset_path;
  std::string dem_path, output_dir_path;
  nh_private.param<std::string>("viewset_path", viewset_path, "");
  nh_private.param<std::string>("dem_path", dem_path, "");
  nh_private.param<std::string>("output_dir_path", output_dir_path, "");

  auto terrain_map = std::make_shared<TerrainMap>();
  terrain_map->Load(dem_path, false);

  /// TODO: Parse exif tags from the images and visualize above DEM
  ESPG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map->getGlobalOrigin(map_coordinate, map_origin);
  terrain_map->AddLayerNormals("elevation");

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
      if (exifio::getViewPointFromImage(image_path, image_name, viewpoint, idx++, map_origin)) {
        viewpoint_list.push_back(viewpoint);
      }
    }
  }

  std::cout << "Viewpoint size: " << viewpoint_list.size() << std::endl;

  colmapio::writePositionsToFile(output_dir_path + "/camera.txt", viewpoint_list);

  ros::spin();
  return 0;
}
