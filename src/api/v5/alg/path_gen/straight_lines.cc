#ifdef PLATFORM_BRAIN

#include "rev/api/v5/alg/path_gen/straight_lines.hh"

using std::cout, std::endl;
using std::shared_ptr;
using std::vector, std::tuple;

namespace rev {

StraightSegments::StraightSegments(
    shared_ptr<Motion> imotion,
    shared_ptr<Correction> icorrection,
    shared_ptr<Stop> istop,
    vector<PointVector> path_points,
    tuple<double, double, double> pid_constants,
    QLength wheelbase,
    size_t resolution,
    QLength look_ahead_distance

    )
    : PurePursuitSegment(imotion,
                         icorrection,
                         istop,
                         pid_constants,
                         wheelbase,
                         look_ahead_distance) {
  this->path_points = path_points;
  this->resolution = resolution == 0 ? resolution : path_points.size() * 5;
}

void StraightSegments::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  path_waypoints = this->generate_waypoints();
  this->current_idx = 0;
  this->last_point = this->path_waypoints.back();
}

std::vector<PointVector> StraightSegments::generate_waypoints() {
  this->path_points.insert(this->path_points.begin(), this->start_point);
  this->res_per_line = this->resolution / this->path_points.size();
  for (size_t i = 0; i < this->path_points.size() - 1; ++i) {
    PointVector start = this->path_points[i];
    PointVector end = this->path_points[i + 1];
    for (size_t j = 0; j < this->res_per_line; ++j) {
      this->path_waypoints.push_back(
        {start.x + (end.x - start.x) * j / this->res_per_line,
         start.y + (end.y - start.y) * j / this->res_per_line}
      );
    }
  }

  return this->path_waypoints;
}

}  // namespace rev

#endif