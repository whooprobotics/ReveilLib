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
#include "rev/util/mathutil.hh"
#include "main.h"
#include "rev/api/alg/drive/stop/simple_stop.hh"



namespace rev {

class BezierSegment : public RecklessSegment{

  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;

  Position start_point;
  std::vector<PointVector> path_points;
  PointVector target_point;
  PointVector prev_point;
  std::size_t current_idx = 1;
  QLength tolerance;
  Position final_point;
  QLength drop_early = 0_in;
  std::vector<PointVector> bezier_points;

  std::size_t resolution; 
  double t_value;

  stop_state new_state;

  SegmentStatus last_status{SegmentStatus::drive(0, 0)};

 public:
  /**
   *
   * @brief Make a new Bezier Segment
   * 
   *
   *
   *
   * 
   */
  BezierSegment(std::vector<PointVector> path_points, std::size_t resolution = 5, QLength tolerance = 2_in,
                std::shared_ptr<Stop> istop = std::make_shared<SimpleStop>(0.1_s, 0.2_s, 0.4));


  void init(OdometryState initial_state) override;

  SegmentStatus step(OdometryState current_state) override;

  void clean_up() override;

};

}