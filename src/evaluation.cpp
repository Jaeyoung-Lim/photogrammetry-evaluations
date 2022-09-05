/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim. All rights reserved.
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

Evaluation::Evaluation() {

}

std::vector<double> Evaluation::calculateErrors(grid_map::GridMap &groundtruth_map,
                                                    const grid_map::GridMap &reference_map) {
  groundtruth_map.add("elevation_difference");
  groundtruth_map["elevation_difference"].setConstant(NAN);

  std::vector<double> error_vector;
  for (grid_map::GridMapIterator iterator(groundtruth_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    Eigen::Vector3d cell_pos_3d;
    bool valid = groundtruth_map.getPosition3("elevation", index, cell_pos_3d);
    if (valid) {
      double cell_elevation = cell_pos_3d(2);
      const Eigen::Vector2d cell_pos = Eigen::Vector2d(cell_pos_3d(0), cell_pos_3d(1));
      /// TODO: Check if this overlaps with the reference map
      /// TODO: Define ROI
      if (reference_map.isInside(cell_pos)) {
        double ref_elevation = reference_map.atPosition("elevation", cell_pos);
        double error = std::abs(ref_elevation - cell_elevation);
        groundtruth_map.at("elevation_difference", index) = error;
        error_vector.push_back(error);
      } else {
        groundtruth_map.at("elevation_difference", index) = NAN;
        error_vector.push_back(NAN);
      }
    }
  }
  return error_vector;
}

double Evaluation::CalculatePrecision(const std::vector<double> &error_vector, const double threshold) {
  int tp_count{0};
  int total_count{0};

  for (auto error_point : error_vector) {
    total_count++;
    if (std::isfinite(error_point) && (std::abs(error_point) < std::abs(threshold))) {
      tp_count++;
    }
  }
  double precision = static_cast<double>(tp_count) / static_cast<double>(total_count);
  return precision;
}

void Evaluation::CompareMapLayer(grid_map::GridMap &groundtruth_map, grid_map::GridMap &reference_map) {
  // Error statistics in Groundtruth -> Reconstructed mapf
  // Calculate Recall
  std::vector<double> error_vector = calculateErrors(groundtruth_map, reference_map);
  // Compute error values
  double cumulative_error{0.0};
  int num_valid_points{0};
  for (auto error_point : error_vector) {
    if (std::isfinite(error_point)) {
      cumulative_error += error_point;
      num_valid_points++;
    }
  }
  double mean = cumulative_error / num_valid_points;
  double cumulative_squared_error{0.0};
  for (auto error_point : error_vector) {
    if (std::isfinite(error_point)) cumulative_squared_error += std::pow(error_point - mean, 2);
  }
  double stdev = std::sqrt(cumulative_squared_error / num_valid_points);
  std::cout << "  - Average Error: " << mean << std::endl;
  std::cout << "  - Average STDEV: " << stdev << std::endl;

  // Error statistics in Reconstructed map -> Groundtruth
  // Calculate Precision
  std::vector<double> error_precision = calculateErrors(reference_map, groundtruth_map);

  /// TODO: Write raw error values into file for ruther analysis
  std::cout << "Elevation Map Error Statistics" << std::endl;

  // Compute error statistics
  {
    double error_threshold = 1.0;
    double precision = CalculatePrecision(error_precision, error_threshold);
    double recall = CalculatePrecision(error_vector, error_threshold);
    double f_score = 2 * precision * recall / (precision + recall);

    std::cout << "  - Precision   (" << error_threshold << " [m]): " << precision << std::endl;
    std::cout << "  - Recall      (" << error_threshold << " [m]): " << recall << std::endl;
    std::cout << "  - F-score     (" << error_threshold << " [m]): " << f_score << std::endl;
  }
  {
    double reference_precision{0.5};
    double accuracy = 0.0;
    int tp_count{0};
    int total_count{0};
    double precision{0.0};
    while (precision < reference_precision) {
      accuracy += 0.1;
      for (auto error_point : error_precision) {
        total_count++;
        if (std::isfinite(error_point) && (std::abs(error_point) < std::abs(accuracy))) {
          tp_count++;
        }
      }
      precision = static_cast<double>(tp_count) / static_cast<double>(total_count);
    }
    std::cout << "  - Accuracy(Precision): " << accuracy << "(" << precision << "[m])" << std::endl;
  }
}
