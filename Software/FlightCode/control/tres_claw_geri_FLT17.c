#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h"
#include "ss_control_interface.h"
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
void alt_hold_inner(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr);
void alt_hold_flutsupHinf(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, int flutsup_init, struct control *controlData_ptr);
void alt_hold_flutsupMidaas(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, int flutsup_init, struct control *controlData_ptr);
void alt_hold_flutsupClas(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, int flutsup_init, struct control *controlData_ptr);
void reset_tracker();
void reset_roll();
void reset_pitch();
void reset_speed();
void reset_alt();
void reset_flutsup();

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

double base_pitch_cmd		= 2.3*D2R;  			// Trim value 2.3 deg @ 23 m/s
double approach_theta 		= -5.5*D2R;				// Absolute angle for the initial approach
double approach_speed 		= 16;					// Approach airspeed, m/s, 20
double flare_theta 			= 1.5*D2R;				// Absolute angle for the flare
double flare_speed 			= 12;					// Flare airspeed, m/s, 17
double pilot_flare_delta	= 2;					// Delta flare airspeed if the pilot is landing, m/s
double trim_speed[3]		= {23, 23, 20};			// Trim airspeed, m/s, 20
double exp_speed[3]     	= {31, 31, 20};	       	// Speed to run the experiments at, m/s
double alt_cmd;
double alt_min              = 30;                   // Minimum altitude hold engage height, m
double alt_max              = 150;                  // Maximum altitude hold engage height, m
double ias_cmd;

double accTrim_cf 			= 0;					// Trim Value
double accTrim_cr 			= 0;					// Trim Value
double accTrim_lf 			= 0;					// Trim Value
double accTrim_lr 			= 0;					// Trim Value
double accTrim_rf 			= 0;					// Trim Value
double accTrim_rr 			= 0;					// Trim Value
double qTrim	 			= 0;					// Trim Value
double azTrim	 			= 0;					// Trim Value

double ss01_input[3] 		= {0, 0, 0};			// Inputs for FlutterSuppression
double ss01_output[2] 		= {0, 0}; 				// Control Deflections for FlutterSuppression
double ss02_input[3] 		= {0, 0, 0};			// Inputs for FlutterSuppression
double ss02_output[2] 		= {0, 0}; 				// Control Deflections for FlutterSuppression
double ss03_input[1] 		= {0};					// Inputs for FlutterSuppression
double ss03_output[1] 		= {0}; 					// Control Deflections for FlutterSuppression
double ss_outputLim			= 8*D2R;				// Limit the output of the flutter suppression

/// *****************************************************************************************

extern void init_control(void) {
	init_ss01_control();
	init_ss02_control();
	init_ss03_control();
};

extern void close_control(void) {
	close_ss01_control();
	close_ss02_control();
	close_ss03_control();
};

extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr) {

	unsigned short claw_mode 	= missionData_ptr -> claw_mode; 	// mode switching
	unsigned short claw_select 	= missionData_ptr -> claw_select; 	// mode switching
	static int t0_latched = FALSE;	// time latching
	static double t0 = 0;
	double claw_time; // time since claw mode started
	static int altCmd_latched = FALSE;	// altitude latching
	static int flutsup_sel = 0;			// flutter suppression mode select, acts as latch for current mode
	static int flutsup_init = FALSE;	// flutter suppression initial
	

	switch(claw_mode){
		case 0: // experiment mode
			t0_latched = FALSE;
			switch(claw_select){
				case 0: // SYSID #1
					missionData_ptr -> run_excitation = 0;
					missionData_ptr -> sysid_select = 0;
					if(altCmd_latched == FALSE){alt_cmd = sensorData_ptr->adData_ptr->h_filt; reset_alt(); altCmd_latched = TRUE;} // Catch first pass to latch current altitude
					if(flutsup_sel != 1){flutsup_init = TRUE; reset_flutsup(); flutsup_sel = 1;} // Catch first pass to latch current sensor values
					
					alt_hold_flutsupMidaas(time, exp_speed[claw_select], alt_cmd, sensorData_ptr, navData_ptr, flutsup_init, controlData_ptr);
					
					if(flutsup_init == TRUE){flutsup_init = FALSE;}
					break;
				case 1: // SYSID #2
					missionData_ptr -> run_excitation = 0;
					missionData_ptr -> sysid_select = 1;
					if(altCmd_latched == FALSE){alt_cmd = sensorData_ptr->adData_ptr->h_filt; reset_alt(); altCmd_latched = TRUE;} // Catch first pass to latch current altitude
					if(flutsup_sel != 2){flutsup_init = TRUE; reset_flutsup(); flutsup_sel = 2;} // Catch first pass to latch current sensor values
					
					alt_hold_flutsupClas(time, exp_speed[claw_select], alt_cmd, sensorData_ptr, navData_ptr, flutsup_init, controlData_ptr);
					
					if(flutsup_init == TRUE){flutsup_init = FALSE;}
					break;
				default: // SYSID #3
					missionData_ptr -> run_excitation = 0;
					missionData_ptr -> sysid_select = 2;
					altCmd_latched = FALSE;
					flutsup_sel = 0;
					
					pilot_flying_inner(time, exp_speed[claw_select], sensorData_ptr, navData_ptr, controlData_ptr);
					
					break;
			}
			break;
		case 2: // autolanding mode
			missionData_ptr -> run_excitation = 0;
			altCmd_latched = FALSE;
			flutsup_sel = 0; flutsup_init = FALSE; flutsup_sel = 0;
			switch(claw_select){
				case 0: // approach
					t0_latched = FALSE;
					approach_control(time, sensorData_ptr, navData_ptr, controlData_ptr);
					break;
				case 1: // flare
					if(t0_latched == FALSE){ // get the time since flare started
						t0 = time;
						t0_latched = TRUE;
					}
					claw_time = time - t0;
					flare_control(claw_time, sensorData_ptr, navData_ptr, controlData_ptr);
					break;
				default: // pilot guidance
					t0_latched = FALSE;
					pilot_flying_inner(time, (flare_speed + pilot_flare_delta), sensorData_ptr, navData_ptr, controlData_ptr);
					break;
			}
			break;
		default: // pilot mode
			missionData_ptr -> run_excitation = 0;
			altCmd_latched = FALSE;
			t0_latched = FALSE;
			switch(flutsup_sel){ // Mode is based on state of flutsup_sel
				default: // Normal, flutter suppression mode has not been latched
					pilot_flying_inner(time, trim_speed[claw_select], sensorData_ptr, navData_ptr, controlData_ptr);
					
					break;
				case 1: // Flutter suppression mode 1
					alt_hold_flutsupMidaas(time, trim_speed[claw_select], alt_cmd, sensorData_ptr, navData_ptr, flutsup_init, controlData_ptr);
					
					break;
				case 2: // Flutter suppression mode 2
					alt_hold_flutsupClas(time, trim_speed[claw_select], alt_cmd, sensorData_ptr, navData_ptr, flutsup_init, controlData_ptr);
					
					break;
			}
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
    controlData_ptr->l2   	= roll_incp*L2_MAX;								    // L2 [rad]
	controlData_ptr->l3   	= -1 * pitch_incp*(L3_MAX-15*D2R);			     	// L3 [rad]
	controlData_ptr->l4   	= 0; 												// L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= 0; 												// R4 [rad]
}

void pilot_flying_inner(double time, double ias_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr);	// phi from the pilot stick + guidance
	controlData_ptr->theta_cmd	= pilot_theta(sensorData_ptr);	// theta from the pilot stick + guidance
	controlData_ptr->ias_cmd = ias_cmd;
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 1);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 1);	// L3 [rad]
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

void alt_hold_inner(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	double alt   = sensorData_ptr->adData_ptr->h_filt;         // altitude
	
	controlData_ptr->phi_cmd   = pilot_phi(sensorData_ptr); // phi from the pilot stick
	controlData_ptr->ias_cmd   = ias_cmd;
	if((alt_cmd > alt_min) & (alt_cmd < alt_max)){
		controlData_ptr->alt_cmd   = alt_cmd;
		controlData_ptr->theta_cmd = alt_control(alt_cmd, alt, TIMESTEP) + pilot_theta(sensorData_ptr); // Altitude tracker + Pilot stick
	}
	else {
		controlData_ptr->alt_cmd   = alt_max; // no effect
		controlData_ptr->theta_cmd = pilot_theta(sensorData_ptr); // Pilot stick
	}
	
	theta_cmd	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= 0;												// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 1);	// L3 [rad]
	controlData_ptr->l4   	= 0; 		                                        // L4 [rad]
    controlData_ptr->r1   	= 0; 												// R1 [rad]
	controlData_ptr->r2   	= -1*controlData_ptr->l2; 							// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= 0; 		                                        // R4 [rad]
}

void alt_hold_flutsupHinf(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, int flutsup_init, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	double alt   = sensorData_ptr->adData_ptr->h_filt;      // altitude
	
	double az     = sensorData_ptr->imuData_ptr->az; 		// IMU Z accel
	double acc_lf   = sensorData_ptr->accelData_ptr->lf;    // Left Forward Accel
	double acc_lr   = sensorData_ptr->accelData_ptr->lr;    // Left Rear Accel
	double acc_rf   = sensorData_ptr->accelData_ptr->rf;    // Right Forward Accel
	double acc_rr   = sensorData_ptr->accelData_ptr->rr;    // Right Rear Accel
	
	controlData_ptr->phi_cmd   = pilot_phi(sensorData_ptr); // phi from the pilot stick
	controlData_ptr->ias_cmd   = ias_cmd;
	if((alt_cmd > alt_min) & (alt_cmd < alt_max)){
		controlData_ptr->alt_cmd   = alt_cmd;
		controlData_ptr->theta_cmd = alt_control(alt_cmd, alt, TIMESTEP) + pilot_theta(sensorData_ptr); // Altitude tracker + Pilot stick
	}
	else {
		controlData_ptr->alt_cmd   = alt_max; // no effect
		controlData_ptr->theta_cmd = pilot_theta(sensorData_ptr); // Pilot stick
	}
	
	theta_cmd	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	if(flutsup_init==TRUE){
		qTrim = q; q = q - qTrim;
		azTrim = az; az = az - azTrim;
		accTrim_lf = acc_lf; acc_lf = acc_lf - accTrim_lf;
		accTrim_lr = acc_lr; acc_lr = acc_lr - accTrim_lr;
		accTrim_rf = acc_rf; acc_rf = acc_rf - accTrim_rf;
		accTrim_rr = acc_rr; acc_rr = acc_rr - accTrim_rr;
	}
	else{
		q = q - qTrim;
		az = az - azTrim;
		acc_lf = acc_lf - accTrim_lf;
		acc_lr = acc_lr - accTrim_lr;
		acc_rf = acc_rf - accTrim_rf;
		acc_rr = acc_rr - accTrim_rr;
	}
	
	ss01_input[0] = q; 		
	ss01_input[1] = az; 		
	ss01_input[2] = 0.25 * (acc_lf + acc_lr + acc_rf + acc_rr);   

	get_ss01_control(ss01_input, ss01_output);
	
	// Limit the output of the ss controller
	if(ss01_output[0] > ss_outputLim) {ss01_output[0] = ss_outputLim;}
	else if((ss01_output[0] < -ss_outputLim) ){ss01_output[0] = -ss_outputLim;}
	if(ss01_output[1] > ss_outputLim) {ss01_output[1] = ss_outputLim;}
	else if((ss01_output[1] < -ss_outputLim) ){ss01_output[1] = -ss_outputLim;}
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= ss01_output[0];										// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 1);	// L3 [rad]
	controlData_ptr->l4   	= ss01_output[1]; 		                            // L4 [rad]
    controlData_ptr->r1   	= controlData_ptr->l1; 								// R1 [rad]
	controlData_ptr->r2   	= -1 * controlData_ptr->l2; 						// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->l4; 		                        // R4 [rad]
}

void alt_hold_flutsupMidaas(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, int flutsup_init, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	double alt   = sensorData_ptr->adData_ptr->h_filt;      // altitude
	
	double acc_cf   = sensorData_ptr->accelData_ptr->cf;    // Center Forward Accel
	double acc_cr   = sensorData_ptr->accelData_ptr->cr;    // Center Rear Accel
	double acc_lf   = sensorData_ptr->accelData_ptr->lf;    // Left Forward Accel
	double acc_lr   = sensorData_ptr->accelData_ptr->lr;    // Left Rear Accel
	double acc_rf   = sensorData_ptr->accelData_ptr->rf;    // Right Forward Accel
	double acc_rr   = sensorData_ptr->accelData_ptr->rr;    // Right Rear Accel
	
	controlData_ptr->phi_cmd   = pilot_phi(sensorData_ptr); // phi from the pilot stick
	controlData_ptr->ias_cmd   = ias_cmd;
	if((alt_cmd > alt_min) & (alt_cmd < alt_max)){
		controlData_ptr->alt_cmd   = alt_cmd;
		controlData_ptr->theta_cmd = alt_control(alt_cmd, alt, TIMESTEP) + pilot_theta(sensorData_ptr); // Altitude tracker + Pilot stick
	}
	else {
		controlData_ptr->alt_cmd   = alt_max; // no effect
		controlData_ptr->theta_cmd = pilot_theta(sensorData_ptr); // Pilot stick
	}
	
	theta_cmd	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	if(flutsup_init==TRUE){
		accTrim_cf = acc_cf; acc_cf = acc_cf - accTrim_cf;
		accTrim_cr = acc_cr; acc_cr = acc_cr - accTrim_cr;
		accTrim_lf = acc_lf; acc_lf = acc_lf - accTrim_lf;
		accTrim_lr = acc_lr; acc_lr = acc_lr - accTrim_lr;
		accTrim_rf = acc_rf; acc_rf = acc_rf - accTrim_rf;
		accTrim_rr = acc_rr; acc_rr = acc_rr - accTrim_rr;
	}
	else{
		acc_cf = acc_cf - accTrim_cf;
		acc_cr = acc_cr - accTrim_cr;
		acc_lf = acc_lf - accTrim_lf;
		acc_lr = acc_lr - accTrim_lr;
		acc_rf = acc_rf - accTrim_rf;
		acc_rr = acc_rr - accTrim_rr;
	}
	
	ss02_input[0] = acc_cf;
	ss02_input[1] = acc_cr;
	ss02_input[2] = 0.25 * (acc_lf + acc_lr + acc_rf + acc_rr);
	
	get_ss02_control(ss02_input, ss02_output);
	
	// Limit the output of the ss controller
	if(ss02_output[0] > ss_outputLim) {ss02_output[0] = ss_outputLim;}
	else if((ss02_output[0] < -ss_outputLim) ){ss02_output[0] = -ss_outputLim;}
	if(ss02_output[1] > ss_outputLim) {ss02_output[1] = ss_outputLim;}
	else if((ss02_output[1] < -ss_outputLim) ){ss02_output[1] = -ss_outputLim;}
	
	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= ss02_output[0];										// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 1);	// L3 [rad]
	controlData_ptr->l4   	= ss02_output[1]; 		                            // L4 [rad]
    controlData_ptr->r1   	= controlData_ptr->l1; 								// R1 [rad]
	controlData_ptr->r2   	= -1 * controlData_ptr->l2; 						// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->l4; 		                        // R4 [rad]
}
void alt_hold_flutsupClas(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, int flutsup_init, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	double alt   = sensorData_ptr->adData_ptr->h_filt;      // altitude
	
	double acc_cf   = sensorData_ptr->accelData_ptr->cf;    // Center Forward Accel
	double acc_cr   = sensorData_ptr->accelData_ptr->cr;    // Center Rear Accel
	
	controlData_ptr->phi_cmd   = pilot_phi(sensorData_ptr); // phi from the pilot stick
	controlData_ptr->ias_cmd   = ias_cmd;
	if((alt_cmd > alt_min) & (alt_cmd < alt_max)){
		controlData_ptr->alt_cmd   = alt_cmd;
		controlData_ptr->theta_cmd = alt_control(alt_cmd, alt, TIMESTEP) + pilot_theta(sensorData_ptr); // Altitude tracker + Pilot stick
	}
	else {
		controlData_ptr->alt_cmd   = alt_max; // no effect
		controlData_ptr->theta_cmd = pilot_theta(sensorData_ptr); // Pilot stick
	}
	
	theta_cmd	= controlData_ptr->theta_cmd;
	phi_cmd		= controlData_ptr->phi_cmd;
	
	if(flutsup_init==TRUE){
		accTrim_cf = acc_cf; acc_cf = acc_cf - accTrim_cf;
		accTrim_cr = acc_cr; acc_cr = acc_cr - accTrim_cr;
	}
	else{
		acc_cf = acc_cf - accTrim_cf;
		acc_cr = acc_cr - accTrim_cr;
	}
	
	ss03_input[0] = (acc_cr - acc_cf);

	get_ss03_control(ss03_input, ss03_output);
	
	// Limit the output of the ss controller
	if(ss03_output[0] > ss_outputLim) {ss03_output[0] = ss_outputLim;}
	else if((ss03_output[0] < -ss_outputLim) ){ss03_output[0] = -ss_outputLim;}

	controlData_ptr->dthr 	= speed_control(ias_cmd, ias, TIMESTEP);			// Throttle [ND 0-1]
	controlData_ptr->l1   	= ss03_output[0];									// L1 [rad]
    controlData_ptr->l2   	= roll_control(phi_cmd, phi, p, TIMESTEP, 0);		// L2 [rad]
	controlData_ptr->l3   	= pitch_control(theta_cmd, theta, q, TIMESTEP, 1);	// L3 [rad]
	controlData_ptr->l4   	= 0;												// L4 [rad]
    controlData_ptr->r1   	= controlData_ptr->l1; 								// R1 [rad]
	controlData_ptr->r2   	= -1 * controlData_ptr->l2; 						// R2 [rad]
	controlData_ptr->r3   	= controlData_ptr->l3;								// R3 [rad]
	controlData_ptr->r4   	= controlData_ptr->l4; 		                        // R4 [rad]
}

void alt_hold(double time, double ias_cmd, double alt_cmd, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double phi_cmd, theta_cmd;
	double alt   = sensorData_ptr->adData_ptr->h_filt;         // altitude
	
	controlData_ptr->phi_cmd   = pilot_phi(sensorData_ptr); // phi from the pilot stick
	controlData_ptr->ias_cmd   = ias_cmd;
	if((alt_cmd > alt_min) & (alt_cmd < alt_max)){
		controlData_ptr->alt_cmd   = alt_cmd;
		controlData_ptr->theta_cmd = alt_control(alt_cmd, alt, TIMESTEP) + pilot_theta(sensorData_ptr); // Altitude tracker + Pilot stick
	}
	else {
		controlData_ptr->alt_cmd   = alt_max; // no effect
		controlData_ptr->theta_cmd = pilot_theta(sensorData_ptr); // Pilot stick
	}
	
	theta_cmd	= controlData_ptr->theta_cmd;
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

void flare_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){
	double ramp_time = 3.5;
	double min_speed = 8;
	double cut_time  = 3;
	double phiLim  = 10 * D2R;
	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	static int t1_latched = FALSE;	// time latching
	static double t1 = 0;
	double ias_cmd, phi_cmd, theta_cmd;
	
	if(time <= ramp_time){ // ramp commands
		ias_cmd = approach_speed - time*((approach_speed - flare_speed)/ramp_time);
		theta_cmd = approach_theta + time*((flare_theta - approach_theta)/ramp_time);
	}
	else{ // final values
		ias_cmd = flare_speed;
		theta_cmd = flare_theta;
	}
	
	if(ias < min_speed){ // engine cutoff
		if(t1_latched == FALSE){
			t1 = time;
			t1_latched = TRUE;
		}
		if((time-t1) > cut_time){
			ias_cmd = -100;	
		}
	}
	else{
		t1_latched = FALSE;
	}
				
	controlData_ptr->theta_cmd 	= theta_cmd - base_pitch_cmd;	// delta theta command [rad]
	controlData_ptr->ias_cmd 	= ias_cmd;						// absolute airspeed command [m/s]
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr);	// phi from the pilot stick;							// wings level
	
	// Reduce the allowable phi commands to +/- phiLim
	if (controlData_ptr->phi_cmd > phiLim){
		controlData_ptr->phi_cmd = phiLim;
	}
	else if(controlData_ptr->phi_cmd < -phiLim){
		controlData_ptr->phi_cmd = -phiLim;
	}
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	ias_cmd		= controlData_ptr->ias_cmd;
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

// approach control, 
void approach_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){

	double phi   = navData_ptr->phi;						// roll angle
	double theta = navData_ptr->the - base_pitch_cmd; 		// subtract theta trim value to convert to delta coordinates
	double p     = sensorData_ptr->imuData_ptr->p; 			// roll rate
	double q     = sensorData_ptr->imuData_ptr->q; 			// pitch rate
	double ias   = sensorData_ptr->adData_ptr->ias_filt;	// filtered airspeed
	double ias_cmd, phi_cmd, theta_cmd;
	
	controlData_ptr->theta_cmd 	= approach_theta - base_pitch_cmd;	// delta theta command [rad]
	controlData_ptr->ias_cmd 	= approach_speed;					// absolute airspeed command [m/s]
	controlData_ptr->phi_cmd	= pilot_phi(sensorData_ptr);		// phi from the pilot stick
	
	theta_cmd 	= controlData_ptr->theta_cmd;
	ias_cmd		= controlData_ptr->ias_cmd;
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
static double alt_control(double alt_ref, double alt, double delta_t)
{
	double theta_cmd;
	double theta_lim = 5.0*D2R;              // Theta command limit from alt hold        

	// Altitude tracker
	e[3] = alt_ref - alt;                         // altitude error
	integrator[3] += e[3]*delta_t*anti_windup[3]; // altitude error integral

	// proportional term  + integral term
	theta_cmd = alt_gain[0]*e[3] + alt_gain[1]*integrator[3];    // Theta Command output

	//eliminate wind-up on altitude integral
	if      (theta_cmd >= theta_lim && e[3] < 0) {anti_windup[3] = 1; theta_cmd = theta_lim;}
	else if (theta_cmd >= theta_lim && e[3] > 0) {anti_windup[3] = 0; theta_cmd = theta_lim;}  //stop integrating
	else if (theta_cmd <= -theta_lim && e[3] < 0) {anti_windup[3] = 0; theta_cmd = -theta_lim;}  //stop integrating
	else if (theta_cmd <= -theta_lim && e[3] > 0) {anti_windup[3] = 1; theta_cmd = -theta_lim;}
	else {anti_windup[3] = 1;}

	return theta_cmd; // non dimensional
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
void reset_flutsup(){
	reset_ss01_control();
	reset_ss02_control();
	reset_ss03_control();
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
	reset_flutsup();

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
