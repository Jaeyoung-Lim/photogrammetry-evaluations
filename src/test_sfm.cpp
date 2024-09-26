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

#include <ros/ros.h>
// For loading the data, see the comments therein for scenario (camera rotates around cube)
#include "photogrammetry_evaluations/SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
// trust-region method known as Powell's Degleg
#include <gtsam/nonlinear/DoglegOptimizer.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include <vector>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {
  ros::init(argc, argv, "sfm_test");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  auto poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
                                                    .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.addPrior(Symbol('x', 0), poses[0], poseNoise);             // add directly to graph

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  for (size_t i = 0; i < poses.size(); ++i) {
    PinholeCamera<Cal3_S2> camera(poses[i], *K);
    for (size_t j = 0; j < points.size(); ++j) {
      Point2 measurement = camera.project(points[j]);
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurement, measurementNoise,
                                                                             Symbol('x', i), Symbol('l', j), K);
    }
  }

  // Because the structure-from-motion problem has a scale ambiguity, the
  // problem is still under-constrained Here we add a prior on the position of
  // the first landmark. This fixes the scale by indicating the distance between
  // the first camera and the first landmark. All other landmark positions are
  // interpreted using this scale.
  auto pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
  graph.addPrior(Symbol('l', 0), points[0],
                 pointNoise);  // add directly to graph
  graph.print("Factor Graph:\n");

  // Create the data structure to hold the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i) {
    auto corrupted_pose = poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20)));
    initialEstimate.insert(Symbol('x', i), corrupted_pose);
  }
  for (size_t j = 0; j < points.size(); ++j) {
    Point3 corrupted_point = points[j] + Point3(-0.25, 0.20, 0.15);
    initialEstimate.insert<Point3>(Symbol('l', j), corrupted_point);
  }
  initialEstimate.print("Initial Estimates:\n");

  /* Optimize the graph and print results */
  Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  result.print("Final results:\n");
  cout << "initial error = " << graph.error(initialEstimate) << endl;
  cout << "final error = " << graph.error(result) << endl;
  ros::spin();
  return 0;
}
