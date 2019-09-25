#include <iostream>
#include <cmath>
#include "Copter.h"

using namespace std;

/*
 * Init and run calls for autonomous flight mode (largely based off of the AltHold flight mode)
 */

// autonomous_init - initialise autonomous controller
bool Copter::autonomous_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    // reset integrators for roll and pitch controllers
    g.pid_roll.reset_I();
    g.pid_pitch.reset_I();

    return true;
}

// autonomous_run - runs the autonomous controller
// should be called at 100hz or more
void Copter::autonomous_run()
{
    AltHoldModeState althold_state;

    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // desired roll, pitch, and yaw_rate
    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif
    target_climb_rate = 0.0f;

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        // compute the target climb rate, roll, pitch and yaw rate
        // land if autonomous_controller returns false
        if (!autonomous_controller(target_climb_rate, target_roll, target_pitch, target_yaw_rate)) {
            // switch to land mode
            set_mode(LAND, MODE_REASON_MISSION_END);
            break;
        }

        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

//===========================================================================//
// START OF OUR AUTONOMOUS CONTROLLER										 //
//===========================================================================//

// this is no longer used
/*
bool equal_within_th(float a, float b, double th) {
	return abs(a - b) < th;
}
*/

/*
	autonomous_controller - Computes the following:
		target climb rate: how fast it should move up/down in cm/sec
		roll: desired roll angle in centi-degrees; positive is to the right
		pitch: desired pitch angle in centi-degrees; positive is upward
		yaw rate: desired angular velocity in centi-degrees/sec; should be const

	Return true to continue flying, and return false to land
*/
bool Copter::autonomous_controller(float &target_climb_rate, float &target_roll,
                                   float &target_pitch, float &target_yaw_rate) {
    // Set desired climb rate in centimeters per second
    target_climb_rate = 0.0f;

    // Set desired yaw rate in centi-degrees (set to zero to hold constant heading)
    target_yaw_rate = 0.0f;

    // Get downward facing sensor reading in meters
    float rangefinder_alt = (float)rangefinder_state.alt_cm / 100.0f;

    // Get horizontal sensor readings in meters
    float dist_forward, dist_right, dist_backward, dist_left;
    g2.proximity.get_horizontal_distance(0, dist_forward);
    g2.proximity.get_horizontal_distance(90, dist_right);
    g2.proximity.get_horizontal_distance(180, dist_backward);
    g2.proximity.get_horizontal_distance(270, dist_left);

    // distance threshold to land (about 0.4 to 0.7 meters)
    float dist_th_l = g.e100_param1;

    // distance threshold to go forward (about 0.5 meters)
    float dist_th_f = g.e100_param2;

    // distance threshold to check left and right (about 0.7 meters)
    float dist_th_lr = g.e100_param3;

    // angle multiplier (about 75 to 300 centidegrees)
	float angle_mult = g.e100_param4;

    // keep the current roll as we go left and right
    static float previous_roll = 0.0f;

    // FIRST: check landing
    //   checks that front, right, and left readings are less than the threshold
    if (dist_forward < dist_th_l && dist_right < dist_th_l && dist_left < dist_th_l) {

        // print debugging message that we are landing
        gcs_send_text(MAV_SEVERITY_INFO, "Landing");
        
        // return false to land
        return false;
    }

    // SECOND: check if you can go forward
    else if (dist_forward > dist_th_f) {

        // print debugging message that we are landing
        gcs_send_text(MAV_SEVERITY_INFO, "Going Forward");

        // change pitch to negative and roll to zero to go forward
        target_pitch = -1.0f * angle_mult;
        target_roll = 0.0f;

        // RECOMMENDATIONS: Use PID controller to go forward
        /*
        // change pitch based on PID and roll to zero to go forward
        g.pid_pitch.set_input_filter_all(dist_th_f - dist_forward);
        target_pitch = 1.0f * angle_mult * g.pid_pitch.get_pid();
        target_roll = 0.0f;
        */
    }

    // THIRD: check if you can go either left or right
    else if (dist_left > dist_th_lr && dist_right > dist_th_lr) {

        // print debugging message that we are maintaining roll
        gcs_send_text(MAV_SEVERITY_INFO, "Maintaining Roll");

        // set roll to previous roll and pitch to zero to continue in same direction
        target_roll = previous_roll;
        target_pitch = 0.0f;
    }

    // FOURTH: check if you can go left
    else if (dist_left > dist_th_lr) {

        // print debugging message that we are going left
        gcs_send_text(MAV_SEVERITY_INFO, "Going Left");

        // change roll to negative and pitch to zero to go left
        target_roll = -1.0f * angle_mult;
        target_pitch = 0.0f;

        // set the previous roll variable to the new target_roll value
        previous_roll = -1.0f * angle_mult;

        // RECOMMENDATIONS: Use PID controller to go left
        /*
        // change pitch to zero and roll based on PID to go left
        g.pid_pitch.set_input_filter_all(dist_th_lr - dist_left);
        target_roll = -1.0f * angle_mult * g.pid_pitch.get_pid();
        target_pitch = 0.0f;
        */
    }

    // FIFTH: check if you can go right
    else if (dist_right > dist_th_lr) {
        // print debugging message that we are going right
        gcs_send_text(MAV_SEVERITY_INFO, "Going Right");

        // change roll to positive and pitch to zero to go right
        target_roll = 1.0f * angle_mult;
        target_pitch = 0.0f;

        // set the previous roll variable to the new target_roll value
        previous_roll = 1.0f * angle_mult;

        // RECOMMENDATIONS: Use PID controller to go right
        /*
        // change pitch to zero and roll based on PID to go right
        g.pid_pitch.set_input_filter_all(dist_th_lr - dist_right);
        target_roll = 1.0f * angle_mult * g.pid_pitch.get_pid();
        target_pitch = 0.0f;
        */
    }

    // Will display target pitch & roll every second (program runs at 400 Hz)
    static int counter = 0;
    if (++counter >= 400) {
        string tp = "target_pitch: " + std::to_string(target_pitch);
        string tr = "target_roll: " + std::to_string(target_roll);
        gcs_send_text(MAV_SEVERITY_INFO, tp.c_str());
        gcs_send_text(MAV_SEVERITY_INFO, tr.c_str());
        gcs_send_text(MAV_SEVERITY_INFO, "============");
        counter = 0;
    }

    // If we aren't landing, always return true to keep flying
    return true;
}


//===========================================================================//
// END OF OUR AUTONOMOUS CONTROLLER											 //
//===========================================================================//

// NOTE: I don't think any of this is needed anymore but I figured we may as well keep it

// ========================== //
// code as of Saturday 4/13  //
// ========================== //

/*
bool Copter::autonomous_controller(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate)
{
    // Set desired climb rate in centimeters per second
    target_climb_rate = 0.0f;

    // Set desired yaw rate in centi-degrees per second (set to zero to hold constant heading)
    target_yaw_rate = 0.0f;

    // Get downward facing sensor reading in meters
    // TODO Are we using this for anything?
    float rangefinder_alt = (float)rangefinder_state.alt_cm / 100.0f;

    // Get horizontal sensor readings in meters
    float dist_front, dist_right, dist_backward, dist_left;
    g2.proximity.get_horizontal_distance(0, dist_front);
    g2.proximity.get_horizontal_distance(90, dist_right);
    g2.proximity.get_horizontal_distance(180, dist_backward);
    g2.proximity.get_horizontal_distance(270, dist_left);

	float angle_mult = 100;

    if (
		equal_within_th(dist_front, 0.5f, 0.1) &&
		equal_within_th(dist_right, 0.5f, 0.1) &&
		(dist_backward == 0 || dist_backward > 2) &&
		equal_within_th(dist_left, 2.5f, 0.2)
	) {
		// At point 1 - takeoff corner (walls: front and right | came from: back)

        gcs_send_text(MAV_SEVERITY_INFO, "Recognizing Point 1");

		target_pitch = 0;
		target_roll = -1 * angle_mult; // 1 degree to the left
	}
	if (
		equal_within_th(dist_front, 1.9f, 0.16) &&
		equal_within_th(dist_right, 2.4f, 0.16) &&
		equal_within_th(dist_backward, 0.5f, 0.1) &&
		equal_within_th(dist_left, 0.5f, 0.1)
	) {
		// At point 2 - first corner (walls: back and left | came from: right)

        gcs_send_text(MAV_SEVERITY_INFO, "Recognizing Point 2");

		target_pitch = -1 * angle_mult;
		target_roll = 0;
	}
	if (
		equal_within_th(dist_front, 0.5f, 0.1) &&
		equal_within_th(dist_right, 2.4f, 0.2) &&
		equal_within_th(dist_backward, 1.1f, 0.2) &&
		equal_within_th(dist_left, 0.5f, 0.1)
	) {
		// At point 3 - second corner (walls: front and left | came from: back)

        gcs_send_text(MAV_SEVERITY_INFO, "Recognizing Point 3");

		target_pitch = 0;
		target_roll = angle_mult;
	}
	if (
		equal_within_th(dist_front, 1.7f, 0.16) &&
		equal_within_th(dist_right, 0.5f, 0.1) &&
		equal_within_th(dist_backward, 0.5f, 0.1) &&
		equal_within_th(dist_left, 2.5f, 0.2)
	) {
		// At point 4 - third/last corner (walls: back and right | came from: left)

        gcs_send_text(MAV_SEVERITY_INFO, "Recognizing Point 4");

		target_pitch = -1 * angle_mult;
		target_roll = 0;
	}
	if (
		equal_within_th(dist_front, 0.5f, 0.1) &&
		equal_within_th(dist_right, 0.5f, 0.1) &&
		equal_within_th(dist_backward, 1.9f, 0.18) &&
		equal_within_th(dist_left, 0.5f, 0.1)
	) {
		// At point 5 - landing zome (walls: front, right, and left | came from: back)

        gcs_send_text(MAV_SEVERITY_INFO, "Recognizing Point 5");

		// Land
		return false;
	}

    // TODO Possible code for dropping the payload
    // TODO Input payload drop command from USB controller

    // Will display target pitch & roll every second (program runs at 400 Hz)
    static int counter = 0;
    if (++counter >= 400) {
        string tp = "target_pitch: " + std::to_string(target_pitch);
        string tr = "target_roll: " + std::to_string(target_roll);
        gcs_send_text(MAV_SEVERITY_INFO, tp.c_str());
        gcs_send_text(MAV_SEVERITY_INFO, tr.c_str());
        gcs_send_text(MAV_SEVERITY_INFO, "============");
        counter = 0;
    }

    return true;
}
*/

// ========================== //
// two new ideas Sunday 4/14  //
// ========================== //

// first: I think instead of equal_within_th we want less than threshold
// I also think we want the threshold value to be accessible from Misison Planner so we can easily modify it
// NOTE: the threshold just adds on to the variable we want to be less than, I think it would be around 0.2

    // example for less_th function
        /*
        bool less_th(float a, float b, double th) {
            return a < (b + th);
        }
        */

    // example for using this function with the first case (inside if statement is the same)
        /*
        float dist_th = g.e100_param1; // NOTE: not sure this is the exact name of the mission planner variable
        if (
                less_th(dist_front, 0.5f, dist_th) &&
                less_th(dist_right, 0.5f, dist_th) &&
                (dist_backward == 0 || dist_backward > 2) &&
                less_th(dist_left, 2.5f, dist_th * 2)
            ) {
                // At point 1 - takeoff corner (walls: front and right | came from: back)
                target_pitch = 0;
                target_roll = -1 * angle_mult; // 1 degree to the left
            }
        */


// second: if we want to rewrite the program to be less hard-coded I think something like this might work
    /*
    // Set desired climb rate in centimeters per second
    target_climb_rate = 0.0f;

    // Set desired yaw rate in centi-degrees per second (set to zero to hold constant heading)
    target_yaw_rate = 0.0f;

    // Get downward facing sensor reading in meters
    float rangefinder_alt = (float)rangefinder_state.alt_cm / 100.0f;

    // Get horizontal sensor readings in meters
    float dist_forward, dist_right, dist_backward, dist_left;
    g2.proximity.get_horizontal_distance(0, dist_forward);
    g2.proximity.get_horizontal_distance(90, dist_right);
    g2.proximity.get_horizontal_distance(180, dist_backward);
    g2.proximity.get_horizontal_distance(270, dist_left);

	// float angle_mult = 100; // Change this to a mission planner variable
    float angle_mult = g.e100_param2;
    
    static string came_from = "backward";

    float dist_th = g.e100_param1; // NOTE: not sure this is the exact name of the mission planner variable
    // ANOTHER NOTE: in this version the distance threshold would be more like 0.4 to 0.7 

    // Case 1: go forward
    if (
            (dist_right < dist_th || came_from == "right") &&
            (dist_backward < dist_th || came_from == "backward") &&
            (dist_left < dist_th || came_from == "left")
        
        ) {
            gcs_send_text(MAV_SEVERITY_INFO, "Going Forward");
            target_pitch = -1 * angle_mult;
            target_roll = 0;
            came_from = "forward";
    }
    
    // Case 2: go right
    if (
            (dist_forward < dist_th || came_from == "forward") &&
            (dist_backward < dist_th || came_from == "backward") &&
            (dist_left < dist_th || came_from == "left")
        
        ) {
            gcs_send_text(MAV_SEVERITY_INFO, "Going Right");
            target_pitch = 0;
            target_roll = angle_mult;
            came_from = "right";
    }

    // Case 3: go backward
    if (
            (dist_forward < dist_th || came_from == "forward") &&
            (dist_right < dist_th || came_from == "right") &&
            (dist_left < dist_th || came_from == "left")
        
        ) {
            gcs_send_text(MAV_SEVERITY_INFO, "Going Backward");
            target_pitch = angle_mult;
            target_roll = 0;
            came_from = "backward";
    }

    // Case 4: go left
    if (
            (dist_forward < dist_th || came_from == "forward") &&
            (dist_right < dist_th || came_from == "right") &&
            (dist_backward < dist_th || came_from == "backward")
        
        ) {
            gcs_send_text(MAV_SEVERITY_INFO, "Going Left");
            target_pitch = 0;
            target_roll = -1 * angle_mult;
            came_from = "left";
    }

    // Case 5: land
    if (
            (dist_forward < dist_th || came_from == "forward") &&
            (dist_right < dist_th || came_from == "right") &&
            (dist_backward < dist_th || came_from == "backward") &&
            (dist_left < dist_th || came_from == "left")
        ) {
            gcs_send_text(MAV_SEVERITY_INFO, "Landing");
            return false;
    }

    // Will display target pitch & roll every second (program runs at 400 Hz)
    static int counter = 0;
    if (++counter >= 400) {
        string tp = "target_pitch: " + std::to_string(target_pitch);
        string tr = "target_roll: " + std::to_string(target_roll);
        gcs_send_text(MAV_SEVERITY_INFO, tp.c_str());
        gcs_send_text(MAV_SEVERITY_INFO, tr.c_str());
        gcs_send_text(MAV_SEVERITY_INFO, "============");
        counter = 0;
    }

    // if we did not land, keep running the script
    return true;
    */

// NOTE ^^ changed to else if's, deleted backwards, and moved landing first
//      still got stuck at the second corner


// OLD CODE: before we switched to the hard-coding method

	/*
    float min_dist = 0.5; // TODO Input as MissionPlanner param?
    //g.pid_pitch.set_input_filter_all(distance[2] - distance[0]); // Back - Front
    g.pid_pitch.set_input_filter_all(
        (distance[2] < min_dist ? (2*min_dist - 2*distance[2]) : 0)
        - (distance[0] < min_dist ? (2*min_dist - 2*distance[0]) : 0)
    );
    target_pitch = 100.0f * g.pid_pitch.get_pid();
    //g.pid_roll.set_input_filter_all(distance[1] - distance[3]); // Right - Left
    g.pid_roll.set_input_filter_all(
        (distance[1] < min_dist ? (2*min_dist - 2*distance[1]) : 0)
        - (distance[3] < min_dist ? (2*min_dist - 2*distance[3]) : 0)
    );
    target_roll = 100.0f * g.pid_roll.get_pid();
	*/