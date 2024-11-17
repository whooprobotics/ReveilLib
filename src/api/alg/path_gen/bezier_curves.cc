#include "rev/api/alg/path_gen/bezier_curves.hh"

using std::cout, std::endl;
namespace rev {

BezierSegment::BezierSegment(std::shared_ptr<Correction> icorrection,
                             std::shared_ptr<Stop> istop,
                             std::vector<PointVector> path_points,
                             double ispeed,
                             std::size_t resolution,
                             QLength tolerance){
  this->correction = icorrection;
  this->path_points = path_points;
  if (resolution == 0) this->resolution = path_points.size() * 3;
  else this->resolution = resolution;
  this->tolerance = tolerance;
  this->stop = istop;
  this->last_point = path_points[path_points.size() - 1];
  this->speed = ispeed;
  // this->approach_distance = approach_distance;
  // this->kpa = kpa;
  // this->kia = kia;
  // this->kda = kda;
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  this->path_points.insert(this->path_points.begin(), this->start_point);
  this->current_idx = 0;
  // std::cout << "AT BEGINNING OF INIT" << std::endl;

  for (std::size_t t = 0; t < this->resolution; ++t) {
    double t_value = static_cast<double>(t) / (this->resolution - 1);
    std::vector<PointVector> temp_points = this->path_points; // Initialize temp_points with path_points

    for (std::size_t layer = 1; layer < this->path_points.size(); ++layer) {
      for (std::size_t idx = 0; idx + layer < this->path_points.size(); ++idx) {
        temp_points[idx] = (1-t_value) * temp_points[idx] + t_value * temp_points[idx + 1];
      }
    }
    this->bezier_points.push_back(temp_points[0]);
  }

  // print out bezier points REMOVE AFTER DEBUG
  // for (std::size_t i = 0; i < this->bezier_points.size(); ++i){
  //   cout << "Bezier Point " << i << ": " << this->bezier_points[i].x.convert(foot) << ", " << this->bezier_points[i].y.convert(foot) << endl;
  // }
}


std::tuple<double, double> calculate_powers(PointVector first_point, Position current_pos, PointVector target_point){
  QLength radius;
  bool left_side_is_inner;
  radius = calculate_radius(first_point, current_pos, target_point);
  //cout << "Radius: " << radius.convert(foot) << endl;
  double outer_power = 1.0;
  double inner_power = outer_power * calculate_inside_ratio(24_in, radius).get_value();
  return left_side_is_inner ? std::make_tuple(inner_power, outer_power):
                              std::make_tuple(outer_power, inner_power);
}

SegmentStatus BezierSegment::step(OdometryState current_state){
  //pros::delay(500);
  //cout << "AT BEGINNING OF STEP" << endl;
  PointVector last_point = this->bezier_points[this->bezier_points.size() - 1];
  //cout << "current state " << current_state.pos.x.convert(foot) << ", " << current_state.pos.y.convert(foot) << endl;
  //cout << "last_point: " << last_point.x.convert(foot) << ", " << last_point.y.convert(foot) << endl;
  //cout << start_point.x.convert(foot) << ", " << start_point.y.convert(foot) << endl;
  //cout << drop_early.convert(foot) << endl;
  new_state = this->stop->get_stop_state(current_state, {last_point.x, last_point.y, 0_deg},
                                                    start_point, this->drop_early);

  //cout << '1' << endl;

  //Prevent status from regressing
  if (last_status.status == SegmentStatusType::NEXT ||
      new_state == stop_state::EXIT)
    return last_status = SegmentStatus::next();

  if (last_status.status == SegmentStatusType::BRAKE ||
      new_state == stop_state::BRAKE)
    return last_status = SegmentStatus::brake();

  //cout << '2' << endl;

  if (current_idx >= this->bezier_points.size()) return SegmentStatus::brake();

  Pose error = current_state.pos.to_relative({this->bezier_points[current_idx].x, this->bezier_points[current_idx].y, 0_deg});
  QLength distance = abs(error.x); // USE LATERAL DISTANCE CALCULATION TO AVOID CIRCLING
  //cout << "Error: " << error.x.convert(foot) << ", " << error.y.convert(foot) << endl;
  
  //cout << "Distance: " << distance.convert(foot) << endl;
  //cout << "Tolerance: " << tolerance.convert(foot) << endl;

  if (distance.convert(foot) < tolerance.convert(foot)){
    ++current_idx;
    cout << "Current Index: " << current_idx << endl;
  }

  //cout << "Current Index: " << current_idx << endl;                           

  target_point = this->bezier_points[current_idx];
  prev_point = this->bezier_points[current_idx-1];


  // cout << "Current Position: " << current_state.pos.x.convert(foot) << ", " << current_state.pos.y.convert(foot) << ", " 
  //      << current_state.pos.theta.convert(degree) << endl;
  // cout << "Target Point: " << target_point.x.convert(foot) << ", " << target_point.y.convert(foot) << endl;
  // cout << "Prev Point: " << prev_point.x.convert(foot) << ", " << prev_point.y.convert(foot) << endl;
  

  std::tuple<double, double> pows = std::make_tuple(this->speed, this->speed);

  // cout << "Current state: " << current_state.pos.x.convert(foot) << ", " << current_state.pos.y.convert(foot) << endl;
  // cout << "Target point: " << target_point.x.convert(foot) << ", " << target_point.y.convert(foot) << endl;
  // cout << "Start point: " << start_point.x.convert(foot) << ", " << start_point.y.convert(foot) << endl;
  // cout << "Drop early: " << drop_early.convert(foot) << endl;
  // cout << "Powers before correction: " << std::get<0>(pows) << ", " << std::get<1>(pows) << endl;

  std::tuple<double, double> corrected_pows =
      this->correction->apply_correction(current_state, {this->target_point.x, this->target_point.y, 0_deg},
                                         this->start_point, this->drop_early,
                                         pows);
  //std::tuple<double, double> pows = calculate_powers(prev_point, current_state.pos, target_point);
  // cout << "Powers: " << std::get<0>(corrected_pows) << ", " << std::get<1>(corrected_pows) << endl;
  // cout << "Current Point: " << current_state.pos.x.convert(foot) << ", " << current_state.pos.y.convert(foot)
  //      << ", " << current_state.pos.theta.convert(degree)<< endl;
  return SegmentStatus::drive(corrected_pows);
}

void BezierSegment::clean_up() {}

}