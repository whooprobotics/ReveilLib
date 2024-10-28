#include "rev/api/alg/path_gen/bezier_curves.hh"

using std::cout, std::endl;
namespace rev {

BezierSegment::BezierSegment(std::vector<PointVector> path_points, std::size_t resolution, QLength tolerance) {
  this->path_points = path_points;
  this->resolution = resolution;
  this->tolerance = tolerance;
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  this->current_idx = 0;
  std::cout << "AT BEGINNING OF INIT" << std::endl;

  std::vector<PointVector> final_points; // Vector to store final interpolated points

  for (std::size_t t = 0; t < this->resolution; ++t) {
    double t_value = static_cast<double>(t) / (this->resolution - 1);
    std::vector<PointVector> temp_points = this->path_points; // Initialize temp_points with path_points

    for (std::size_t layer = 1; layer < this->path_points.size(); ++layer) {
      for (std::size_t idx = 0; idx + layer < this->path_points.size(); ++idx) {
        cout << t << "    " << idx << "   " << layer << "            "
         << temp_points[idx].x.convert(foot)
         << "    " << temp_points[idx + 1].x.convert(foot) << endl;

        temp_points[idx] = (1-t_value) * temp_points[idx] + t_value * temp_points[idx + 1];

        cout << t << "    " << idx << "   " << layer << "            "
         << temp_points[idx].x.convert(foot)
         << "    " << temp_points[idx + 1].x.convert(foot) << endl;
        
        cout << endl;

      }
    }

    final_points.push_back(temp_points[0]);
    cout << "FINAL   " << temp_points[0].x.convert(foot) << endl;;
  }

}


std::tuple<double, double> calculate_powers(PointVector first_point, Position current_pos, PointVector target_point){
  QLength radius;
  bool left_side_is_inner;
  radius = calculate_radius(first_point, current_pos, target_point);
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
  prev_point = this->path_points[current_idx-1];

  if (current_idx >= this->path_points.size()) return SegmentStatus::brake();

  QLength distance = sqrt((current_state.pos.x - target_point.x)*(current_state.pos.x - target_point.x) + 
                          (current_state.pos.y - target_point.y)*(current_state.pos.y - target_point.y));

  if (distance < tolerance) ++current_idx;

  std::tuple<double, double> pows = calculate_powers(prev_point, current_state.pos, target_point);
  return SegmentStatus::drive(pows);
}

void BezierSegment::clean_up() {}

}