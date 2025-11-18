
#include "rev/api/alg/drive/callback/callback.hh"

namespace rev {

callback::callback(std::function<void()> auton_callback,
					          float percent)
		: auton_callback(auton_callback),
		  percent(percent) {}

bool callback::progress(float current_percent) {
	if(current_percent >= percent && !called) {
		auton_callback();
		called = true;
		return true;
	}
	return false;
}

} // namespace rev




