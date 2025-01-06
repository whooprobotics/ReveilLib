#pragma once

#include <iostream>
#include <cstddef>
#include <memory>
#include <vector>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/pure_pursuit/pure_pursuit.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"
#include "rev/api/alg/drive/motion/motion.hh"
#include "rev/api/alg/drive/correction/correction.hh"
#include "rev/api/alg/drive/stop/stop.hh"
#include "main.h"
#include "rev/api/alg/drive/stop/simple_stop.hh"

namespace rev {

class BezierSegment : public PurePursuitSegment {
public:
  /**
   *
   * @brief Make a new Bezier Segment
   * @note O(p^2 * r) where p is the number of path points and r is the resolution
   * 
   * @param icorrection The correction for pilons segment
   * @param istop The stop for pilons segment
   * @param path_points The points for the bezier curve
   * @param resolution The resolution of the bezier curve, if left at 0 becomes length of path points times 3.
   * It is important not to set the resolution too high as it can become too computationally intensive
   *
   */
  BezierSegment(std::shared_ptr<Motion> imotion,
                std::shared_ptr<Correction> icorrection,
                std::shared_ptr<Stop> istop,
                std::vector<PointVector> path_points,
                std::tuple<double, double, double> pid_constants,
                QLength wheelbase,
                std::size_t resolution = 0,
                QLength look_ahead_distance = 1_ft                
                );
  
  /**
   * @brief Initialize the Bezier segment by generating waypoints
   * 
   * @param initial_state The initial odometry state of the robot
   */
  void init(OdometryState initial_state) override;

protected:
  /**
   * @brief Generate the Bezier curve waypoints based on control points
   * 
   * @return std::vector<PointVector> Generated waypoints
   */
  std::vector<PointVector> generate_waypoints() override;

private:
  std::shared_ptr<Motion> motion;
  std::shared_ptr<Correction> correction;
  std::shared_ptr<Stop> stop;
  std::vector<PointVector> path_points;

  Position target_point;
  Position final_point;
  QLength wheelbase;
  QLength look_ahead_distance;

  std::size_t resolution; 
  double t_value;


  double left_speed;
  double right_speed;



};

}