#include "mecanum_PID.hh"
#include "mecanum_util.hh"
#include <cmath>

PID::PID() {};

PID::PID(double kp, double ki, double kd, double starti) :
    kp(kp),
    ki(ki),
    kd(kd),
    starti(starti)
{};

PID::PID(double kp, double ki, double kd, double starti, double settle_error, double settle_time, double large_settle_error, double large_settle_time, double exit_error, double timeout) :
    kp(kp),
    ki(ki),
    kd(kd),
    starti(starti),
    settle_error(settle_error),
    settle_time(settle_time),
    large_settle_error(large_settle_error),
    large_settle_time(large_settle_time),
    exit_error(exit_error),
    timeout(timeout)
{};

double PID::compute(double error) {
    if (fabs(error) < starti){
        accumulated_error += error;
    }
    if (sign(error) != sign(previous_error)) { 
        accumulated_error = 0; 
    }

    output = kp * error + ki * accumulated_error + kd * (error - previous_error);

    previous_error = error;

    if(fabs(error) < settle_error) {
        time_spent_settled += 10;
    } else {
        time_spent_settled = 0;
    }

    if (fabs(error) < large_settle_error) {
        time_spent_large_settled += 10;
    } else {
        time_spent_large_settled = 0;
    }

    if (fabs(error) < exit_error && exit_error != 0) {
        exiting = true;
    }

    time_spent_running += 10;

    return output;
}

bool PID::is_settled(){
    if (time_spent_running > timeout && timeout != 0) {
        return true;
    }
    if (time_spent_settled > settle_time || time_spent_large_settled > large_settle_time) {
        return true;
    }
    if (exiting) {
        exiting = false;
        return true;
    }
    return false;
}