struct DriveParams;
struct TurnToAngleParams;

class ControlReckless {
public:
    void set_default_stop_constants(float harsh_threshold_ms, float coast_threshold_ms, float coast_power, float timeout_ms = 0.0);
    void set_default_drive_constants(float max_power, float correction, float max_error);
    void set_default_turn_constants(float max_power, float coast_power, float harsh_coeff, float coast_coeff, float brake_time_ms);

    // Stop constants
    float stop_harsh_threshold_ms;
    float stop_coast_threshold_ms;
    float stop_coast_power;
    float stop_timeout_ms;
    
    // Drive constants
    float drive_max_power;
    float drive_correction;
    float drive_max_error;

    // Turn constants
    float turn_max_power;
    float turn_coast_power;
    float turn_harsh_coeff;
    float turn_coast_coeff;
    float turn_brake_time_ms;

    ControlReckless();

    void drive_to_point(float x, float y, const DriveParams& p);
    void turn_to_angle(float angle_deg, const TurnToAngleParams& p);
};

extern ControlReckless control;

struct DriveParams{
    float max_power = control.drive_max_power;
    float correction = control.drive_correction;
    float max_error = control.drive_max_error;
};

struct TurnToAngleParams{
    float max_power = control.turn_max_power;
    float coast_power = control.turn_coast_power;
    float harsh_coeff = control.turn_harsh_coeff;
    float coast_coeff = control.turn_coast_coeff;
    float brake_time_ms = control.turn_brake_time_ms;
};

