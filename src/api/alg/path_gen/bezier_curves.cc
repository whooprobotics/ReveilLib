#include "rev/api/alg/path_gen/bezier_curves.hh"

using std::cout, std::endl;
namespace rev {

BezierSegment::BezierSegment(std::shared_ptr<Motion> imotion,
                             std::shared_ptr<Correction> icorrection,
                             std::shared_ptr<Stop> istop,
                             std::vector<PointVector> path_points,
                             std::size_t resolution,
                             QLength tolerance,
                             QLength wheelbase,
                             QLength look_ahead_distance
  ): PurePursuitSegment(imotion, icorrection, istop, look_ahead_distance, wheelbase, tolerance)                     
  {
  this->motion = imotion;
  this->correction = icorrection;
  this->stop = istop;
  this->path_points = path_points;
  if (resolution == 0) this->resolution = path_points.size() * 3;
  else this->resolution = resolution;
  this->tolerance = tolerance;
  this->wheelbase = wheelbase;
  this->look_ahead_distance = look_ahead_distance;
  
  this->last_point = path_points[path_points.size() - 1];
}

void BezierSegment::init(OdometryState initial_state) {
  this->start_point = initial_state.pos;
  path_waypoints = generate_waypoints();
  this->current_idx = 0;
}

std::vector<PointVector> BezierSegment::generate_waypoints() {
  //this->start_point = initial_state.pos;
  //this->path_points.insert(this->path_points.begin(), this->start_point);
  this->current_idx = 0;

  for (std::size_t t = 0; t < this->resolution; ++t) {
    double t_value = static_cast<double>(t) / (this->resolution - 1);
    std::vector<PointVector> temp_points = this->path_points; // Initialize temp_points with path_points

    for (std::size_t layer = 1; layer < this->path_points.size(); ++layer) {
      for (std::size_t idx = 0; idx + layer < this->path_points.size(); ++idx) {
        temp_points[idx] = (1-t_value) * temp_points[idx] + t_value * temp_points[idx + 1];
      }
    }
    this->path_waypoints.push_back(temp_points[0]);
  }

  return this->path_waypoints;
  //print out bezier points REMOVE AFTER DEBUG
  // for (std::size_t i = 0; i < this->path_waypoints.size(); ++i){
  //   cout << "Bezier Point " << i << ": " << this->path_waypoints[i].x.convert(inch) << ", " << this->path_waypoints[i].y.convert(inch) << endl;
  //}
}

// SegmentStatus BezierSegment::step(OdometryState current_state){
//   PointVector last_point = this->path_waypoints[this->path_waypoints.size() - 1];
//   new_state = this->stop->get_stop_state(current_state, {last_point.x, last_point.y, 0_deg},
//                                                     start_point, this->drop_early);

//   //Prevent status from regressing
//   if (last_status.status == SegmentStatusType::NEXT ||
//       new_state == stop_state::EXIT)
//     return last_status = SegmentStatus::next();

//   if (last_status.status == SegmentStatusType::BRAKE ||
//       new_state == stop_state::BRAKE)
//     return last_status = SegmentStatus::brake();

//   if (current_idx >= this->path_waypoints.size()) return SegmentStatus::brake();

//   Pose error = current_state.pos.to_relative({this->path_waypoints[current_idx].x, this->path_waypoints[current_idx].y, 0_deg});
//   QLength distance = abs(error.x); // USE LATERAL DISTANCE CALCULATION TO AVOID CIRCLING

//   if (distance.convert(foot) < tolerance.convert(foot)){
//     ++current_idx;
//     cout << "ERROR: " << error.x.convert(inch) << "in, " << error.y.convert(inch) << "in" << endl;
//     cout << "Index is now: " << current_idx << endl;
//   }

//   target_point = {this->path_waypoints[current_idx].x, this->path_waypoints[current_idx].y, current_state.pos.theta};
//   prev_point = this->path_waypoints[current_idx-1];

//   std::tuple<double, double> pows = this->motion->gen_powers(
//       current_state, target_point, this->start_point, this->drop_early);

//   // Handle coasting if needed
//   if (new_state == stop_state::COAST) {
//     double power = this->stop->get_coast_power();
//     double left, right;
//     std::tie(left, right) = pows;
//     if (left + right < 0)
//       power *= -1;
//     return last_status = SegmentStatus::drive(power);
//   }

//   std::tuple<double, double> corrected_pows =
//       this->correction->apply_correction(current_state, {this->target_point.x, this->target_point.y, current_state.pos.theta},
//                                          this->start_point, this->drop_early,
//                                          pows);


//   return SegmentStatus::drive(corrected_pows);
// }

// void BezierSegment::clean_up() {}

} // namespace rev