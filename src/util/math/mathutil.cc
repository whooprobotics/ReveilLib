#include "rev/util/mathutil.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"

namespace rev{

QLength calculate_radius(Position first_point, PointVector next_coords){
  QAngle current_heading = first_point.theta;
  PointVector first_coords = {first_point.x, first_point.y};
  PointVector midpoint_coords = {(first_coords.x + next_coords.x)/2.0, (first_coords.y + next_coords.y)/2.0};

  RQuantity coords_heading_slope = (next_coords.y - first_coords.y) / (next_coords.x - first_coords.x);
  RQuantity perp_bisector_slope = (-1.0)/coords_heading_slope;
  RQuantity perp_heading = tan(current_heading + 90_deg);
  
  PointVector center;
  center.x = (perp_heading * first_coords.x - perp_bisector_slope * midpoint_coords.x + midpoint_coords.y - first_coords.y)
             / (perp_heading - perp_bisector_slope);
  center.y = perp_bisector_slope * (center.x - midpoint_coords.x) + midpoint_coords.y;
  
  QLength radius = sqrt((center.x - first_coords.x)*(center.x - first_coords.x) + (center.y - first_coords.y)*(center.y - first_coords.y));
  
  return radius;

}

}