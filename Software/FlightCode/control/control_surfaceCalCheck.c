#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"

#include "../system_id/systemid_interface.h"

// ***********************************************************************************
#include AIRCRAFT_UP1DIR            // for Flight Code
// ***********************************************************************************

/// Definition of local functions: ****************************************************
static double pilot_theta(struct sensordata *sensorData_ptr);
static double pilot_phi(struct sensordata *sensorData_ptr);

static double roll_control (double phi_ref, double phi_meas, double rollrate, double delta_t, unsigned short gain_selector);
static double pitch_control(double the_ref, double the_meas, double pitchrate, double delta_t, unsigned short gain_selector);
static double speed_control(double speed_ref, double airspeed, double delta_t);
static double alt_control(double alt_ref, double alt, double delta_t);

void approach_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void flare_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void pilot_flying(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void pilot_flying_inner(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void open_loop(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void alt_hold(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void reset_tracker();
void reset_roll();
void reset_pitch();
void reset_speed();
void reset_alt();

// initialize pitch and roll angle tracking errors, integrators, and anti wind-up operators
// 0 values correspond to the roll tracker, 1 values correspond to the theta tracker, 2 values to the speed tracker, 3 values to the altitude tracker
static double e[4] = {0,0,0};
static double integrator[4] = {0,0,0,0};
static short anti_windup[4]={1,1,1,1};   // integrates when anti_windup is 1

/// ****************************************************************************************

/// Gains
#ifdef AIRCRAFT_SKOLL
	static double roll_gain[3]  		= {0.5,0.15,0.01};  	// PI gains for roll tracker and roll damper
	static double roll_gain_single[3]  	= {1.5,0.5,0.0};  		// PI gains for roll tracker and roll damper when using only Flap2
	static double pitch_gain[3] 		= {-0.3,-0.40,-0.00};  	// PI gains for pitch tracker and pitch damper
	static double pitch_gain_single[3] 	= {-0.75,-1.0,-0.0};  	// PI gains for pitch tracker and pitch damper when using only Flap3
	static double v_gain[2]     		= {0.1, 0.020};			// PI gains for speed tracker
#endif

#ifdef AIRCRAFT_HATI
	static double roll_gain[3]  		= {0.5,0.15,0.01};  	// PI gains for roll tracker and roll damper
	static double roll_gain_single[3]  	= {1.5,0.5,0.0};  		// PI gains for roll tracker and roll damper when using only Flap2
	static double pitch_gain[3] 		= {-0.3,-0.40,-0.00};  	// PI gains for pitch tracker and pitch damper
	static double pitch_gain_single[3] 	= {-0.75,-1.0,-0.0};  	// PI gains for pitch tracker and pitch damper when using only Flap3
	static double v_gain[2]     		= {0.1, 0.020};			// PI gains for speed tracker
#endif

#ifdef AIRCRAFT_GERI
	static double roll_gain[3]  		= {0.5,0.15,0.01};  	// PI gains for roll tracker and roll damper
	static double roll_gain_single[3]  	= {0.5,0.15,0.01};  	// PI gains for roll tracker and roll damper when using only Flap2
	static double pitch_gain[3] 		= {-0.3,-0.15,0.0};  	// PI gains for pitch tracker and pitch damper
	static double pitch_gain_single[3] 	= {-0.3,-0.15,0.0};  	// PI gains for pitch tracker and pitch damper when using only Flap3
	static double v_gain[2]     		= {0.0278, 0.0061};		// PI gains for speed tracker
	static double alt_gain[2]     		= {0.0543*D2R, 0.0*D2R};// PI gains for speed tracker
#endif

double base_pitch_cmd		= 0.0;  				// Trim value 4 deg

double angleUp = 10.0*D2R;
double angleDown = -10.0*D2R;
double angleZero = 0.0*D2R;
double angleCmd;

/// *****************************************************************************************

extern void init_control(void) {
};

extern void close_control(void) {
};

extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {

	unsigned short claw_mode 	= missionData_ptr -> claw_mode; 	// mode switching
	unsigned short claw_select 	= missionData_ptr -> claw_select; 	// mode switching
	
	switch(claw_mode){
		case 0: // Check Calibrations
			switch(claw_select){
				case 0:
					angleCmd = angleDown;
					break;
				case 1:
					angleCmd = angleZero;
					break;
				default:
					angleCmd = angleUp;
					break;
			}
			
			controlData_ptr->l1   = angleCmd;	// L1 [rad]
			controlData_ptr->l2   = angleCmd;	// L2 [rad]
			controlData_ptr->l3   = angleCmd;	// L3 [rad]
			controlData_ptr->l4   = angleCmd; 	// L4 [rad]
			controlData_ptr->r1   = angleCmd; 	// R1 [rad]
			controlData_ptr->r2   = angleCmd; 	// R2 [rad]
			controlData_ptr->r3   = angleCmd; 	// R3 [rad]
			controlData_ptr->r4   = angleCmd; 	// R4 [rad]			
	
			break;
		case 2: // Open Loop
			
			pilot_flying(time, 0, sensorData_ptr, navData_ptr, controlData_ptr);
			
			add_trim_bias(controlData_ptr);  // Add trim biases to get_control surface commands
			break;

		default: // pilot mode
			open_loop(time, 0, sensorData_ptr, navData_ptr, controlData_ptr);
			break;
	}
}

static double pilot_theta(struct sensordata *sensorData_ptr){
	double pitch_incp = sensorData_ptr->inceptorData_ptr->pitch;
	double theta_cmd;
	
	theta_cmd = 30*D2R*pitch_incp;
	return theta_cmd;
}

static double pilot_phi(struct sensordata *sensorData_ptr){
	double roll_incp  = sensorData_ptr->inceptorData_ptr->roll;
	double phi_cmd;
	
	phi_cmd = 35*D2R*roll_incp;
	return phi_cmd;
}

void open_loop(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double roll_incp  = sensorData_ptr->inceptorData_ptr->roll;
	double pitch_incp = sensorData_ptr->inceptorData_ptr->pitch;

	controlData_ptr->ias_cmd = ias_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= -roll_incp*L2_MAX;								// L2 [rad]
	controlData_ptr->l3   	= 1*pitch_incp*(L3_MAX-15*D2R);	     				// L3 [rad]
	controlData_ptr->l4   	= 0; 												// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= 0; 												// R4 [rad]
}

void pilot_flying(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr);		// phi from the pilot stick
	controlData_ptr->theta_cmd	= pilot_theta(sensorData_ptr);		// theta from the pilot stick
	controlData_ptr->ias_cmd = ias_cmd;
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 0);	// L3 [rad]
	controlData_ptr->l4   	= controlData_ptr->l3 + controlData_ptr->l2; 		// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->r3 + controlData_ptr->r2; 		// R4 [rad]
}

// Roll get_control law: angles in radians. Rates in rad/s. Time in seconds
static double roll_control (double phi_ref, double phi_meas, double rollrate, double delta_t, unsigned short gain_selector)
{
	double da;

	// roll attitude tracker
	e[0] = phi_ref - phi_meas;
	integrator[0] += e[0]*delta_t*anti_windup[0]; //roll error integral (rad)

	//proportional term + integral term              - roll damper term
	if(gain_selector == 1){
		da  = roll_gain_single[0]*e[0] + roll_gain_single[1]*integrator[0] - roll_gain_single[2]*rollrate;
	}
	else{
		da  = roll_gain[0]*e[0] + roll_gain[1]*integrator[0] - roll_gain[2]*rollrate;
	}
	//eliminate windup
	if      (da >= L2_MAX-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 1; da = L2_MAX-ROLL_SURF_TRIM;}
	else if (da >= L2_MAX-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 0; da = L2_MAX-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= L2_MIN-ROLL_SURF_TRIM && e[0] < 0) {anti_windup[0] = 0; da = L2_MIN-ROLL_SURF_TRIM;}  //stop integrating
	else if (da <= L2_MIN-ROLL_SURF_TRIM && e[0] > 0) {anti_windup[0] = 1; da = L2_MIN-ROLL_SURF_TRIM;}
	else {anti_windup[0] = 1;}

    return da;
}

// Pitch get_control law: angles in radians. Rates in rad/s. Time in seconds
static double pitch_control(double the_ref, double the_meas, double pitchrate, double delta_t, unsigned short gain_selector)
{
	double de;
	
	// pitch attitude tracker
	e[1] = the_ref - the_meas;
	integrator[1] += e[1]*delta_t*anti_windup[1]; //pitch error integral

    // proportional term + integral term               - pitch damper term
	if(gain_selector == 1){
		de = pitch_gain_single[0]*e[1] + pitch_gain_single[1]*integrator[1] - pitch_gain_single[2]*pitchrate;    // Elevator output
	}
	else{
		de = pitch_gain[0]*e[1] + pitch_gain[1]*integrator[1] - pitch_gain[2]*pitchrate;    // Elevator output
	}
		
	//eliminate wind-up
	if      (de >= L3_MAX-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 0; de = L3_MAX-PITCH_SURF_TRIM;}  //stop integrating
	else if (de >= L3_MAX-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 1; de = L3_MAX-PITCH_SURF_TRIM;}
	else if (de <= L3_MIN-PITCH_SURF_TRIM && e[1] < 0) {anti_windup[1] = 1; de = L3_MIN-PITCH_SURF_TRIM;}
	else if (de <= L3_MIN-PITCH_SURF_TRIM && e[1] > 0) {anti_windup[1] = 0; de = L3_MIN-PITCH_SURF_TRIM;}  //stop integrating
	else {anti_windup[1] = 1;}

	return de;  //rad
}

static double speed_control(double speed_ref, double airspeed, double delta_t)
{
	double dthr;
	
	// Speed tracker
	e[2] = speed_ref - airspeed;
	integrator[2] += e[2]*delta_t*anti_windup[2]; // airspeed error integral

	// proportional term  + integral term
	dthr = v_gain[0]*e[2] + v_gain[1]*integrator[2];    // Throttle output

	//eliminate wind-up on airspeed integral
	if      (dthr >= THROTTLE_MAX-THROTTLE_TRIM && e[2] < 0) {anti_windup[2] = 1; dthr = THROTTLE_MAX-THROTTLE_TRIM;}
	else if (dthr >= THROTTLE_MAX-THROTTLE_TRIM && e[2] > 0) {anti_windup[2] = 0; dthr = THROTTLE_MAX-THROTTLE_TRIM;}  //stop integrating
	else if (dthr <= THROTTLE_MIN-THROTTLE_TRIM && e[2] < 0) {anti_windup[2] = 0; dthr = THROTTLE_MIN-THROTTLE_TRIM;}  //stop integrating
	else if (dthr <= THROTTLE_MIN-THROTTLE_TRIM && e[2] > 0) {anti_windup[2] = 1; dthr = THROTTLE_MIN-THROTTLE_TRIM;}
	else {anti_windup[2] = 1;}

	return dthr; // non dimensional
}


void reset_roll(){
	e[0] = 0;
	integrator[0] = 0;
	anti_windup[0] = 1;
}
void reset_pitch(){
	e[1] = 0;
	integrator[1] = 0;
	anti_windup[1] = 1;
}
void reset_speed(){
	e[2] = 0;
	integrator[2] = 0;
	anti_windup[2] = 1;
}
void reset_alt(){
	e[3] = 0;
	integrator[3] = 0;
	anti_windup[3] = 1;
}

void reset_tracker(){
	reset_roll();
	reset_pitch();
	reset_speed();
	reset_alt();
}

// Reset parameters to initial values
extern void reset_control(struct control *controlData_ptr){

	reset_tracker();

	controlData_ptr->dthr = 0; 	// throttle
	controlData_ptr->l1   = 0;	// L1 [rad]
	controlData_ptr->l2   = 0;	// L2 [rad]
	controlData_ptr->l3   = 0;	// L3 [rad]
	controlData_ptr->l4   = 0; 	// L4 [rad]
	controlData_ptr->r1   = 0; 	// R1 [rad]
	controlData_ptr->r2   = 0; 	// R2 [rad]
	controlData_ptr->r3   = 0; 	// R3 [rad]
	controlData_ptr->r4   = 0; 	// R4 [rad]
}
