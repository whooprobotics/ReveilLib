#pragma once

#include "rev/rev.hh"

// Drive Algs
void driveTo(rev::QLength distance, rev::Drive params = {});
void driveTo(rev::QLength x, rev::QLength y, rev::Drive params = {});
void driveTo(rev::QLength x, rev::QLength y, rev::QAngle angle, rev::Drive params = {});
void turnTo(rev::QAngle angle, rev::Turn params = {});
void turnTo(rev::QLength x, rev::QLength y, rev::Turn params = {});

// drive code
extern bool field_centric_enabled;
void drive();

// Antijam state
extern bool is_sorting;
extern bool is_stalled;
extern bool front_intake_stalled;
extern bool back_intake_stalled;

// Intake state flags
extern bool toggle_intake;
extern bool eject_state;
extern bool outtake_state;
extern bool intake_in_state;

// scoring state flags
extern bool score;
extern bool score_shallow;
extern bool lever_actuating;
extern bool lever_retracting;
extern uint32_t score_press_time;
extern bool lever_score_fail;
extern bool lever_retract_fail;
extern bool lift_state;


// Functions
void anti_jam();
void reset_jam();

// Color enum for color sorting
enum Color {
  RED = 0,
  BLUE = 1,
  NONE = 2
};

Color detect_color();
void kill_sorting(bool state);
bool is_sorting_on();
void set_team(Color team_color);
Color get_team();
void color_sort(Color color, Color team_color);
void color_task();


// Intake functions
void intake_control();
void intake(bool state);
void outtake(bool state);
void eject(bool state);
void stop_intake();
void intake_task();

// State setters
void set_scraper(bool state);
void set_descore(bool state);
void set_lift(bool state);
void set_hood(bool state);

// lever logic
void reset_lever();
void lever_control();
bool score_lever(bool score_shallo = false, bool retry = false);
void recover_lever();
