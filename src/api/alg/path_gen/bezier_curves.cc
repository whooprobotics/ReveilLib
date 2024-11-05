#include "rev/api/alg/path_gen/bezier_curves.hh"

using std::cout, std::endl;
namespace rev {

BezierSegment::BezierSegment(std::vector<PointVector> path_points, std::size_t resolution, QLength tolerance, 
                             std::shared_ptr<Stop> istop) {
  this->path_points = path_points;
  this->resolution = resolution;
  this->tolerance = tolerance;
  this->stop = istop;
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
        temp_points[idx] = (1-t_value) * temp_points[idx] + t_value * temp_points[idx + 1];
      }
    }
    final_points.push_back(temp_points[0]);
  }
}


std::tuple<double, double> calculate_powers(PointVector first_point, Position current_pos, PointVector target_point){
  QLength radius;
  bool left_side_is_inner;
  radius = calculate_radius(first_point, current_pos, target_point);
  cout << "Radius: " << radius.convert(foot) << endl;
  double outer_power = 1.0;
  double inner_power = outer_power * calculate_inside_ratio(24_in, radius).get_value();
  return left_side_is_inner ? std::make_tuple(inner_power, outer_power):
                              std::make_tuple(outer_power, inner_power);
}



SegmentStatus BezierSegment::step(OdometryState current_state){
  //pros::delay(500);
  cout << "AT BEGINNING OF STEP" << endl;
  PointVector last_point = this->path_points[this->path_points.size() - 1];
  cout << "current state " << current_state.pos.x.convert(foot) << ", " << current_state.pos.y.convert(foot) << endl;
  cout << "last_point: " << last_point.x.convert(foot) << ", " << last_point.y.convert(foot) << endl;
  cout << start_point.x.convert(foot) << ", " << start_point.y.convert(foot) << endl;
  cout << drop_early.convert(foot) << endl;
  new_state = this->stop->get_stop_state(current_state, {last_point.x, last_point.y, 0_deg},
                                                    start_point, this->drop_early);

  cout << '1' << endl;

  //Prevent status from regressing
  if (last_status.status == SegmentStatusType::NEXT ||
      new_state == stop_state::EXIT)
    return last_status = SegmentStatus::next();

  if (last_status.status == SegmentStatusType::BRAKE ||
      new_state == stop_state::BRAKE)
    return last_status = SegmentStatus::brake();

  cout << '2' << endl;                                              

  target_point = this->path_points[current_idx];
  prev_point = this->path_points[current_idx-1];

  cout << "Current Position: " << current_state.pos.x.convert(foot) << ", " << current_state.pos.y.convert(foot) << ", " 
       << current_state.pos.theta.convert(degree) << endl;
  cout << "Target Point: " << target_point.x.convert(foot) << ", " << target_point.y.convert(foot) << endl;
  cout << "Prev Point: " << prev_point.x.convert(foot) << ", " << prev_point.y.convert(foot) << endl;
  

  if (current_idx >= this->path_points.size()) return SegmentStatus::brake();

  QLength distance = sqrt((current_state.pos.x - target_point.x)*(current_state.pos.x - target_point.x) + 
                          (current_state.pos.y - target_point.y)*(current_state.pos.y - target_point.y));
  
  cout << "Distance: " << distance.convert(foot) << endl;

  if (distance < tolerance) ++current_idx;

  std::tuple<double, double> pows = calculate_powers(prev_point, current_state.pos, target_point);
  cout << "Powers: " << std::get<0>(pows) << ", " << std::get<1>(pows) << endl;
  return SegmentStatus::drive(pows);
}

void BezierSegment::clean_up() {}

}