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

#ifndef EXIF_IO_H
#define EXIF_IO_H

#include "GeographicLib/Geoid.hpp"
#include "adaptive_viewutility/adaptive_viewutility.h"
#include "photogrammetry_evaluations/geo_conversions.h"

namespace exifio {
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

}  // namespace exifio

#endif
