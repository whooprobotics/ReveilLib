#include "rev/api/alg/path_gen/bezier_curves.hh"

namespace rev {

BezierSegment::BezierSegment(std::vector<PointVector> path_points, std::size_t resolution, QLength tolerance) {
  this->path_points = path_points;
  this->resolution = resolution;
  this->tolerance = tolerance;
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  this->current_idx = 0;

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


std::tuple<double, double> calculate_powers(Position current_point, PointVector target_point){
  QLength radius;
  bool left_side_is_inner;
  std::tie(radius, left_side_is_inner) = calculate_radius(current_point, target_point);
  double outer_power = 1.0;
  double inner_power = outer_power * calculate_inside_ratio(24_in, radius).get_value();
  return left_side_is_inner ? std::make_tuple(inner_power, outer_power):
                              std::make_tuple(outer_power, inner_power);
}



SegmentStatus BezierSegment::step(OdometryState current_state){
  PointVector last_point = this->path_points[this->path_points.size() - 1];
  new_state = this->stop->get_stop_state(current_state, {last_point.x, last_point.y, 0_deg},
                                                    start_point, this->drop_early);

  // Prevent status from regressing
  if (last_status.status == SegmentStatusType::NEXT ||
      new_state == stop_state::EXIT)
    return last_status = SegmentStatus::next();

  if (last_status.status == SegmentStatusType::BRAKE ||
      new_state == stop_state::BRAKE)
    return last_status = SegmentStatus::brake();                                              

  target_point = this->path_points[current_idx];

  if (current_idx >= this->path_points.size()) return SegmentStatus::brake();

  QLength distance = sqrt((current_state.pos.x - target_point.x)*(current_state.pos.x - target_point.x) + 
                          (current_state.pos.y - target_point.y)*(current_state.pos.y - target_point.y));

  if (distance < tolerance) ++current_idx;

  std::tuple<double, double> pows = calculate_powers(current_state.pos, target_point);
  return SegmentStatus::drive(pows);
}

void BezierSegment::clean_up() {}

}