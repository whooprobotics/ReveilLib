#include "rev/api/alg/drive/stop/stop.hh"
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/alg/path_gen/bezier_curves.hh"
#include <cstddef>
#include <vector>


namespace rev {

BezierSegment::BezierSegment(std::vector<PointVector> path_points, std::size_t resolution){
  this->path_points = path_points;
  this->resolution = resolution;
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;

  for (std::size_t t = 0; t < this->resolution; ++t){
    t_value = (double) t / resolution;
    for (std::size_t layer = 1; layer < this->path_points.size(); ++layer){
      for (std::size_t idx = 0; layer + idx < this->path_points.size(); ++idx){
        this->path_points[idx].x = (1 - t_value) * this->path_points[idx].x + t_value * this->path_points[idx + 1].x;
        this->path_points[idx].y = (1 - t_value) * this->path_points[idx].y + t_value * this->path_points[idx + 1].y;
      }
    }
  }

}

SegmentStatus BezierSegment::step(OdometryState current_state){
  // stop_state new_state = this->stop->get_stop_state(current_state, target_point,
  //                                                   start_point, drop_early);
}

void BezierSegment::clean_up() {}

}