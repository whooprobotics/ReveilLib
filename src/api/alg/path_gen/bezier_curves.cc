#include "rev/api/alg/path_gen/bezier_curves.hh"

using std::cout, std::endl;
namespace rev {

BezierSegment::BezierSegment(std::shared_ptr<Motion> imotion,
                             std::shared_ptr<Correction> icorrection,
                             std::shared_ptr<Stop> istop,
                             std::vector<PointVector> path_points,
                             std::tuple<double, double, double> pid_constants,
                             QLength wheelbase,
                             std::size_t resolution,
                             QLength look_ahead_distance

                             )
    : PurePursuitSegment(imotion,
                         icorrection,
                         istop,
                         pid_constants,
                         wheelbase,
                         look_ahead_distance) {
  this->path_points = path_points;
  this->resolution = resolution == 0 ? resolution : path_points.size() * 3;
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  path_waypoints = this->generate_waypoints();
  this->current_idx = 0;
  this->last_point = this->path_waypoints.back();
}

std::vector<PointVector> BezierSegment::generate_waypoints() {
  this->path_points.insert(this->path_points.begin(), this->start_point);
  this->current_idx = 0;

  for (std::size_t t = 0; t < this->resolution; ++t) {
    double t_value = static_cast<double>(t) / (this->resolution - 1);
    std::vector<PointVector> temp_points =
        this->path_points;  // Initialize temp_points with path_points

    for (std::size_t layer = 1; layer < this->path_points.size(); ++layer) {
      for (std::size_t idx = 0; idx + layer < this->path_points.size(); ++idx) {
        temp_points[idx] =
            (1 - t_value) * temp_points[idx] + t_value * temp_points[idx + 1];
      }
    }
    this->path_waypoints.push_back(temp_points[0]);
  }

  return this->path_waypoints;
}

}  // namespace rev