
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "systemid_interface.h"

#include AIRCRAFT_UP1DIR

int overspeed = TRUE;
double overspeed_limit = 2.0; // overspeed threshold [m/s]
int indx;
double timeStart;
double timeExcite;
double timeEnd;

extern void get_system_id(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){
	
	if (controlData_ptr->ias_cmd < (sensorData_ptr->adData_ptr->ias_filt - overspeed_limit)){
		overspeed = TRUE;
	}
	else{
		overspeed = FALSE;
	}
	
	indx = (int) (time*150.0);
	
	switch(missionData_ptr -> sysid_select){
		case 0: // SysID#1
		timeStart = 6.0;
		timeExcite = 8.0;
		timeEnd = timeStart + timeExcite;
		
			if (time < timeStart){
			}
			else if((time >= timeStart) && (time < timeEnd)){
				controlData_ptr->surf3_excite = playback_OMS1_1_8(indx - timeStart*150, 0.0*D2R);
				if(overspeed == FALSE) {
					controlData_ptr->l3    += controlData_ptr->surf3_excite;
					controlData_ptr->r3    += controlData_ptr->surf3_excite;
				}
			}
			
			controlData_ptr->cmp_status = time / (timeEnd) * 100.0;
			break;
		case 1: // SysID#2
		timeStart = 6.0;
		timeExcite = 8.0;
		timeEnd = timeStart + timeExcite;
		
			if (time < timeStart){
			}
			else if((time >= timeStart) && (time < timeEnd)){
				controlData_ptr->surf3_excite = playback_OMS1_1_8(indx - timeStart*150, 0.0*D2R);
				if(overspeed == FALSE) {
					controlData_ptr->l3    += controlData_ptr->surf3_excite;
					controlData_ptr->r3    += controlData_ptr->surf3_excite;
				}
			}
			
			controlData_ptr->cmp_status = time / (timeEnd) * 100.0;
			break;
		case 2: // SysID#3
		timeStart = 6.0;
		timeExcite = 8.0;
		timeEnd = timeStart + timeExcite;
		
			if (time < timeStart){
			}
			else if((time >= timeStart) && (time < timeEnd)){
				controlData_ptr->surf3_excite = playback_OMS1_1_8(indx - timeStart*150, 0.0*D2R);
				if(overspeed == FALSE) {
					controlData_ptr->l3    += controlData_ptr->surf3_excite;
					controlData_ptr->r3    += controlData_ptr->surf3_excite;
				}
			}
			
			controlData_ptr->cmp_status = time / (timeEnd) * 100.0;
			break;
	}
}
