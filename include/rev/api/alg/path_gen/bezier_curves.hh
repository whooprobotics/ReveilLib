#pragma once
#include <cstddef>
#include <memory>
#include <vector>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/util/math/point_vector.hh"
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/stop/stop.hh"


namespace rev {

class BezierSegment : public RecklessSegment{

  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;

  Position start_point;
  std::vector<PointVector> path_points;
  std::size_t resolution; 
  double t_value;

 public:
  /**
   *
   * @brief Make a new Bezier Segment
   * 
   * @param idfk i dont know whats happening
   *
   *
   *
   */
  BezierSegment(std::vector<PointVector>, std::size_t resolution = 100);


  void init(OdometryState initial_state) override;

  SegmentStatus step(OdometryState current_state) override;

  void clean_up() override;

};

}