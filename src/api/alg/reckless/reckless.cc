#include "rev/api/alg/reckless/reckless.hh"
namespace rev {
    void Reckless::step() {
        // TODO: define step function
    }

    /**
     * This function starts the robot along a path
    */
    void Reckless::go(RecklessPath path) {
        if(!is_completed())
            breakout();
        current_path = path;
        status = RecklessStatus::ACTIVE;
    }

    /**
     * This function returns the current status of the controller
    */
    RecklessStatus Reckless::get_status() {
        return status;
    }
    
    /**
     * This function gets the current progress along the total path. [0,1] for the first segment, [1,2] second segment, etc. Returns
     * -1.0 if the controller is not running. Returns the integer upper bound of a motion if that motion has invoked a harsh stop
    */
    double Reckless::progress() {
        // TODO: Implement progress
        return 0.0;
    }
    /**
     * This function returns true if the status is DONE, and false otherwise
    */
    bool Reckless::is_completed() {
        return get_status() == RecklessStatus::DONE;
    }
    /**
     * This function immediately sets the status to DONE and ends the current motion
    */
    void Reckless::breakout() {
        status = RecklessStatus::DONE;
    }
}