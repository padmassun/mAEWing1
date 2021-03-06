
#ifndef GLOBALDEFS_H_
#define GLOBALDEFS_H_

// ****** Macro function definitions *******************************************
#define	mymin(arg1,arg2) 	(arg1<=arg2 ? arg1:arg2) 	///< Return the lesser of two input arguments */
#define	mymax(arg1,arg2)	(arg1>=arg2 ? arg1:arg2)	///< Return the greater of two input arguments */
#define sign(arg) 			(arg>=0 ? 1:-1) 			///< Return the sign of the input argument */
// *****************************************************************************

// ******  Thread Settings *****************************************************
#define TIMESTEP 0.02 ///< Base time step, needed for control laws */
// *****************************************************************************

// ****** Unit conversions and constant definitions: ***************************
#define NSECS_PER_SEC	1000000000 		///< [nsec/sec] nanoseconds per second */
#define D2R			0.017453292519943	///< [rad] degrees to radians */
#define R2D			57.295779513082323	///< [deg] radians to degrees */
#define PSI_TO_KPA  6.89475729  		///< [KPa] PSI to KPa */
#define	g			9.814				///< [m/sec^2] gravity */
#define g2      	19.62   			///< [m/sec^2] 2*g */
#define PI      	3.14159265358979    ///< pi */
#define PI2     	6.28318530717958	///< pi*2 */
#define half_pi		1.57079632679490	///< pi/2 */
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif	
// *****************************************************************************

// ****** Type definitions ***************************************************** 
typedef unsigned char	byte;	///< typedef of byte */
typedef unsigned short	word;	///< typedef of word */

/// Define status message enum list
enum   errdefs	{
	got_invalid,		///< No data received
	checksum_err,		///< Checksum mismatch
	gps_nolock,			///< No GPS lock
	data_valid,			///< Data valid
	noPacketHeader,		///< Some data received, but cannot find packet header
	incompletePacket,	///< Packet header found, but complete packet not received
	TU_only,			///< NAV filter, time update only
	gps_aided,			///< NAV filter, GPS aided
	};

/// IMU Data Structure
struct imu {
	double p;	///< [rad/sec], body X axis angular rate (roll)
	double q;	///< [rad/sec], body Y axis angular rate (pitch)
	double r;	///< [rad/sec], body Z axis angular rate (yaw)
	double ax;	///< [m/sec^2], body X axis acceleration
	double ay;	///< [m/sec^2], body Y axis acceleration
	double az;	///< [m/sec^2], body Z axis acceleration
	double hx;	///< [Gauss], body X axis magnetic field
	double hy;	///< [Gauss], body Y axis magnetic field
	double hz;	///< [Gauss], body Z axis magnetic field
	double phi; ///< [rad], Euler roll angle. Only used if IMU sensor reports attitude.
	double the; ///< [rad], Euler pitch angle. Only used if IMU sensor reports attitude.
	double psi; ///< [rad], Euler yaw angle. Only used if IMU sensor reports attitude.
	float  T;	///< [degC], temperature of IMU sensor
	float  Vs;	///< [Volts], supply voltage of IMU sensor
	double adc; ///< [counts], ADC reading
	enum errdefs err_type; ///< IMU status
	double time; ///< [sec], timestamp of IMU data
};

/// GPS Data Structure
struct gps {
	double lat;	///< [deg], Geodetic latitude
	double lon;	///< [deg], Geodetic longitude
	double alt;	///< [m], altitude relative to WGS84
	double alt_prev;
	double ve;	///< [m/sec], East velocity
	double vn;	///< [m/sec], North velocity
	double vd;	///< [m/sec], Down velocity
	double Xe;	///< [m], X position, ECEF
	double Ye;	///< [m], Y position, ECEF
	double Ze;	///< [m], Z position, ECEF
	double Ue;	///< [m/sec], X velocity, ECEF
	double Ve;	///< [m/sec], Y velocity, ECEF
	double We;	///< [m/sec], Z velocity, ECEF
    double sig_N; ///< [m], Position error standard deviation in the North direction
    double sig_E; ///< [m], Position error standard deviation in the East direction
    double sig_D; ///< [m], Position error standard deviation in the Down direction
    double sig_vn; ///< [m/sec], Velocity error standard deviation in the North direction
    double sig_ve; ///< [m/sec], Velocity error standard deviation in the East direction
    double sig_vd; ///< [m/sec], Velocity error standard deviation in the Down direction
	double GPS_TOW;	///< [sec], GPS Time Of Week
	//double courseOverGround;///< [rad], course over the ground, relative to true North
	//double speedOverGround;	///< [rad], speed over the ground
	double time;	///< [sec], timestamp of GPS data
	double update;
	double hAcc;
	//double vAcc;
	//double sAcc;
	//double headingAcc;
	//double pDOP;
	unsigned short newData;	///< [bool], flag set when GPS data has been updated
	unsigned short satVisible; ///< Number satellites used in the position solution
	unsigned short navValid;///< flag indicating whether the solution is valid, 0 = valid
	unsigned short GPS_week;///< GPS week since current epoch.
	enum errdefs err_type;	///< GPS status
	int baudRate;		///< Baud rate for serial port
	char* portName;		///< Name of serial port
	int port;			///< handle for accessing serial port
    unsigned char* localBuffer; ///< local buffer to store partial serial data packets
    int bytesInLocalBuffer; ///< number of bytes in the local buffer
    int readState;			///< current state of serial data reader function
    int read_calls;			///< number of times the read_gps function has been called
};

/// Rabbit Data Structure
struct rabbit {
	double lf;	///< left front accel
	double lr;	///< left rear accel
	double cf;	///< center front accel
	double cr;	///< center rear accel
	double rf;	///< right front accel
	double rr;	///< right rear accel
	double Ps;	///< [KPa], static pressure
	double Pd;	///< [KPa], dynamic pressure
	double pwm_ch_one;
	double pwm_ch_two;
	double pwm_ch_three;
	double pwm_ch_four;
	unsigned short mode;
	unsigned short isUpdated;
	unsigned short satVisible;
	double lat;	///< [deg], Geodetic latitude
	double lon;	///< [deg], Geodetic longitude
	double alt;	///< [m], altitude relative to WGS84
	//double courseOverGround;///< [rad], course over the ground, relative to true North
	//double speedOverGround;	///< [rad], speed over the ground
	double navvn;
	double navve;
	double navvd;
	double hAcc;
	//double vAcc;
	//double sAcc;
	//double headingAcc;
	//double pDOP;
	int baudRate;		///< Baud rate for serial port
	char* portName;		///< Name of serial port
	int port;			///< handle for accessing serial port
};

/// Air Data Structure
struct airdata {
	double h;		///< [m], barometric altitude above ground level (AGL)
	double ias;     ///< [m/sec], indicated airspeed
	double h_filt;	///< [m], filtered altitude
	double ias_filt;	///< [m/s], filtered airspeed
	double Ps;		///< [KPa], static pressure
	double Pd;		///< [KPa], dynamic pressure
	double bias[10];///< array for storing biases for air data.
	unsigned short status;	///< status bitfield for air data sensors.
};

/// Pilot inceptor Data structure
struct inceptor {
	double select;		///< select command from the pilot, ND
	double pitch;		///< pitch stick command from the pilot, ND
	double mode;		///< mode command from the pilot, ND
	double roll;		///< roll stick command from the pilot, ND
};

/// Accelerometer Data structure
struct accel {
	double lf;	///< left front accel
	double lr;	///< left rear accel
	double cf;	///< center front accel
	double cr;	///< center rear accel
	double rf;	///< right front accel
	double rr;	///< right rear accel
};

/// Mission manager Data structure
struct mission {
	unsigned short mode;			///< mode variable; 0 = dump data, 1 = manual control, 2 = autopilot control
	unsigned short run_num;			///< counter for number of autopilot engagements
	unsigned short run_excitation;  ///< mode variable; 0 = no excitation, 1 = start excitations
	unsigned short claw_mode;		///< mode variable; 3 position depending on claw to run
	unsigned short claw_select;		///< mode variable; 3 position depending on claw to run
	unsigned short sysid_select;	///< mode variable; depending on sysid to run
	unsigned short recording;
	int loop;
};

/// Control Data structure
struct control {
	double dthr;		///< [0-1], throttle command
	double l1; 			///< [rad], L1 command, +TED
	double l2; 			///< [rad], L2 command, +TED
	double l3; 			///< [rad], L3 command, +TED
	double l4; 			///< [rad], L4 command, +TED
	double r1;			///< [rad], R1 command, +TED
	double r2;			///< [rad], R2 command, +TED
	double r3;			///< [rad], R3 command, +TED
	double r4;			///< [rad], R4 command, +TED
	double surf1_excite;
	double surf2_excite;
	double surf3_excite;
	double surf4_excite;
	double pitch_cmd_pilot;
	double pitch_cmd_damper;
	double pitch_cmd_excite;
	double roll_cmd_pilot;
	double roll_cmd_damper;
	double roll_cmd_excite;
	double phi_cmd;
	double theta_cmd;
	double ias_cmd;
	double init_alt;
	double alt_cmd;
	double zdot_cmd;
	double cmp_status;
};

/// Navigation Filter Data Structure
struct nav {
	double lat;		///< [rad], geodetic latitude estimate
	double lon;		///< [rad], geodetic longitude estimate
	double alt;		///< [m], altitude relative to WGS84 estimate
	double vn;		///< [m/sec], north velocity estimate
	double ve;		///< [m/sec], east velocity estimate
	double vd;		///< [m/sec], down velocity estimate
	double phi;		///< [rad], Euler roll angle estimate
	double the;		///< [rad], Euler pitch angle estimate
	double psi;		///< [rad], Euler yaw angle estimate
	double quat[4];	///< Quaternions estimate
	double ab[3];	///< [m/sec^2], accelerometer bias estimate
	double gb[3];	///< [rad/sec], rate gyro bias estimate
	double asf[3];	///< [m/sec^2], accelerometer scale factor estimate
	double gsf[3];	///< [rad/sec], rate gyro scale factor estimate
	double Pp[3];	///< [rad], covariance estimate for position
	double Pv[3];	///< [rad], covariance estimate for velocity
	double Pa[3];	///< [rad], covariance estimate for angles
	double Pab[3];	///< [rad], covariance estimate for accelerometer bias
	double Pgb[3];	///< [rad], covariance estimate for rate gyro bias
	double Pasf[3];	///< [rad], covariance estimate for accelerometer scale factor
	double Pgsf[3];	///< [rad], covariance estimate for rate gyro scale factor
	enum errdefs err_type;	///< NAV filter status
	double time;			///< [sec], timestamp of NAV filter
};

/// Combined sensor data structure
struct sensordata {
	struct imu *imuData_ptr; 			///< pointer to imu data structure
	struct gps *gpsData_ptr;			///< pointer to gps data structure
	struct gps *gpsData_l_ptr;			///< pointer to left gps data structure
	struct gps *gpsData_r_ptr;			///< pointer to right gps data structure
	struct airdata *adData_ptr;			///< pointer to airdata data structure
	struct inceptor *inceptorData_ptr;	///< pointer to accelerometer data structure
	struct accel *accelData_ptr;
	struct rabbit *rabbitData_ptr;
};

/// Datalogging data structure
struct datalog {
	char** saveAsDoubleNames;		///< pointer to char array of variable names for doubles
	double** saveAsDoublePointers;	///< pointer to double pointer array to variables that will be saved as doubles
	char** saveAsFloatNames;		///< pointer to char array of variable names for floats
	double** saveAsFloatPointers;	///< pointer to double pointer array to variables that will be saved as floats
	char** saveAsIntNames;			///< pointer to char array of variable names for ints
	int** saveAsIntPointers;		///< pointer to int32_t pointer array to variables that will be saved as ints
	char** saveAsShortNames;		///< pointer to char array of variable names for shorts
	unsigned short** saveAsShortPointers;	///< pointer to uint16_t pointer array to variables that will be saved as shorts
	int logArraySize; 	///< Number of data points in the logging array. 50 Hz * 60 sec/min * 30 minutes = 90000
	int numDoubleVars;	///< Number of variables that will be logged as doubles
	int numFloatVars;	///< Number of variables that will be logged as floats
	int numIntVars;		///< Number of variables that will be logged as ints
	int numShortVars;	///< Number of variables that will be logged as shorts
};
#endif	/* GLOBALDEFS_H_ */
