#pragma once

struct point {
  double x;
  double y;
};

// Importing odom that uses right hand cartesian coordinates with 0 degrees pointing up and positive angles being clockwise
class Odom45 {
public:
  void set_physical_distances(double horizontal_distance_from_cog, double vertical_distance_from_cog);
	void set_position(point position, double orientation_deg, double right_tracker_position, double left_tracker_position);
	void update_position(double right_tracker_position, double left_tracker_position, double orientation_deg);

    double get_x() { return this->position.x; }
    double get_y() { return this->position.y; }

	point position;
	double orientation_deg;

private:
	double horizontal_distance_from_cog;
	double vertical_distance_from_cog;

	double right_tracker_position;
	double left_tracker_position;
};