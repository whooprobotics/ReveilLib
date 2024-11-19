#pragma once

#include <iostream>
#include <cstddef>
#include <memory>
#include <vector>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/stop/stop.hh"
#include "main.h"
#include "rev/api/alg/drive/stop/simple_stop.hh"

namespace rev {

class BezierSegment : public RecklessSegment{

  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;
  std::vector<PointVector> path_points;
  PointVector last_point;
  double speed;

  std::vector<PointVector> bezier_points;

  Position start_point;
  PointVector target_point;
  PointVector prev_point;
  std::size_t current_idx = 1;
  QLength tolerance;
  Position final_point;
  QLength drop_early = 0_in;

  std::size_t resolution; 
  double t_value;

  stop_state new_state;

  SegmentStatus last_status{SegmentStatus::drive(0, 0)};

  double left_speed;
  double right_speed;

 public:
  /**
   *
   * @brief Make a new Bezier Segment
   * @note O(p^2 * r) where p is the number of path points and r is the resolution
   * 
   * @param icorrection The correction for pilons segment
   * @param istop The stop for pilons segment
   * @param path_points The points for the bezier curve
   * @param ispeed The maximum speed, default is 70%
   * @param resolution The resolution of the bezier curve, if left at 0 becomes length of path points times 3.
   * It is important not to set the resolution too high as it can become too computationally intensive
   * @param tolerance The lateral tolerance between each point on the bezier curve
   *
   */
  BezierSegment(std::shared_ptr<Correction> icorrection,
                std::shared_ptr<Stop> istop,
                std::vector<PointVector> path_points,
                double ispeed = 0.7,
                std::size_t resolution = 0,
                QLength tolerance = 1_in
                );

  /**
   * @brief Initialize the segment
   * 
   * @param initial_state The initial state of the robot
   * @details This function computes points which will approximate a bezier curve of the given points.
   * PilonsCorrection will be used to drive to each point generated
   */
  void init(OdometryState initial_state) override;

  /**
   * @brief Step function for the segment
   * 
   * @param current_state The current position of the robot
   * @return SegmentStatus
   * 
   * @details This function will:
   *   1. Check if the segment is completed
   *   2. Index through the bezier points
   *   3. Adjust the robot power with pilons correction
   */
  SegmentStatus step(OdometryState current_state) override;

  void clean_up() override;

};

}