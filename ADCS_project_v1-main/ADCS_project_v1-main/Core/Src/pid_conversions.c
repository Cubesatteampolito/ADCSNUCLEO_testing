//This is the c file for implementation of functions regarding the conversion between torque and pwm

#include "pid_conversions.h"


const float N_spires[3] = {300, 300, 210};       //[], x,y,z
const float A_torquers[3] = {0.01, 0.01, 0.01};     //m^2, x,y,z
const float torquer_Req_Ohm[3] = {30.7, 30.7, 23};     //OHMs, x,y,z
const float torquer_Vdd[3] = {11.77, 11.77, 11.77};         //V_dd , x,y,z
uint8_t flag = 1;


void PID_attitude_error_calculation(PID_Inputs_struct *PID_Inputs){

//  PID_Inputs->accell_Error[0] = PID_Inputs->accell_Desired[0] - PID_Inputs->accell_Measured[0];
//  PID_Inputs->accell_Error[1] = PID_Inputs->accell_Desired[1] - PID_Inputs->accell_Measured[1];
//  PID_Inputs->accell_Error[2] = PID_Inputs->accell_Desired[2] - PID_Inputs->accell_Measured[2];

  PID_Inputs->angSpeed_Error[0] = PID_Inputs->angSpeed_Desired[0] - PID_Inputs->angSpeed_Measured[0];
  PID_Inputs->angSpeed_Error[1] = PID_Inputs->angSpeed_Desired[1] - PID_Inputs->angSpeed_Measured[1];
  PID_Inputs->angSpeed_Error[2] = PID_Inputs->angSpeed_Desired[2] - PID_Inputs->angSpeed_Measured[2];

  printf(" PID_Inputs->angSpeed_Error[0]: %f \n",PID_Inputs->angSpeed_Error[0]);
  printf(" PID_Inputs->angSpeed_Error[1]: %f \n",PID_Inputs->angSpeed_Error[1]);
  printf(" PID_Inputs->angSpeed_Error[2]: %f \n",PID_Inputs->angSpeed_Error[2]);
}


void PID_attitude_Derivative_calculation(PID_Inputs_struct *PID_Inputs)
{
//    PID_Inputs->d_Accell_Err_dt[0] = (PID_Inputs->accell_Measured[3] - PID_Inputs->accell_Measured[0]) / PID_Inputs->timestamp;
//    PID_Inputs->d_Accell_Err_dt[1] = (PID_Inputs->accell_Measured[4] - PID_Inputs->accell_Measured[1]) / PID_Inputs->timestamp;
//    PID_Inputs->d_Accell_Err_dt[2] = (PID_Inputs->accell_Measured[5] - PID_Inputs->accell_Measured[2]) / PID_Inputs->timestamp;
	if(!flag)
	{
		PID_Inputs->d_AngSpeed_Err_dt[0] = (PID_Inputs->angSpeed_Error[3] - PID_Inputs->angSpeed_Error[0]) / PID_Inputs->timestamp;
		PID_Inputs->d_AngSpeed_Err_dt[1] = (PID_Inputs->angSpeed_Error[4] - PID_Inputs->angSpeed_Error[1]) / PID_Inputs->timestamp;
		PID_Inputs->d_AngSpeed_Err_dt[2] = (PID_Inputs->angSpeed_Error[5] - PID_Inputs->angSpeed_Error[2]) / PID_Inputs->timestamp;
	}
	else
	{
		flag = 0;
		PID_Inputs->d_AngSpeed_Err_dt[0] = 0;
		PID_Inputs->d_AngSpeed_Err_dt[1] = 0;
		PID_Inputs->d_AngSpeed_Err_dt[2] = 0;
	}

    printf(" PID_Inputs->d_AngSpeed_Err_dt[0]: %f \n",PID_Inputs->d_AngSpeed_Err_dt[0]);
    printf(" PID_Inputs->d_AngSpeed_Err_dt[1]: %f \n",PID_Inputs->d_AngSpeed_Err_dt[1]);
    printf(" PID_Inputs->d_AngSpeed_Err_dt[2]: %f \n",PID_Inputs->d_AngSpeed_Err_dt[2]);
}

/*

void PID_current_error_calculation(PID_Inputs_struct *PID_Inputs){

    PID_Inputs->current_Error[0] = PID_Inputs->current_Desired[0] - PID_Inputs->current_Measured[0];
    PID_Inputs->current_Error[1] = PID_Inputs->current_Desired[1] - PID_Inputs->current_Measured[1];
    PID_Inputs->current_Error[2] = PID_Inputs->current_Desired[2] - PID_Inputs->current_Measured[2];

}


void PID_current_Derivative_calculation(PID_Inputs_struct *PID_Inputs, const uint8_t timestamp){

    PID_Inputs->d_Current_Mes_dt[0] = (PID_Inputs->current_Measured[3] - PID_Inputs->current_Measured[0]) / timestamp;
    PID_Inputs->d_Current_Mes_dt[1] = (PID_Inputs->current_Measured[4] - PID_Inputs->current_Measured[1]) / timestamp;
    PID_Inputs->d_Current_Mes_dt[2] = (PID_Inputs->current_Measured[5] - PID_Inputs->current_Measured[2]) / timestamp;

}

*/


void PID_angSpeed_error_2_torque(PID_Inputs_struct *PID_Inputs){


    PID_Inputs->Torque_required[0] = PID_Inputs->P_Gain[0] * PID_Inputs->angSpeed_Error[0] + PID_Inputs->D_Gain[0] * PID_Inputs->d_AngSpeed_Err_dt[0];

    PID_Inputs->Torque_required[1] = PID_Inputs->P_Gain[1] * PID_Inputs->angSpeed_Error[1] + PID_Inputs->D_Gain[1] * PID_Inputs->d_AngSpeed_Err_dt[1];

    PID_Inputs->Torque_required[2] = PID_Inputs->P_Gain[2] * PID_Inputs->angSpeed_Error[2] + PID_Inputs->D_Gain[2] * PID_Inputs->d_AngSpeed_Err_dt[2];

    printf(" PID_Inputs->Torque_required[0]: %f \n",PID_Inputs->Torque_required[0]);
    printf(" PID_Inputs->Torque_required[1]: %f \n",PID_Inputs->Torque_required[0]);
    printf(" PID_Inputs->Torque_required[2]: %f \n",PID_Inputs->Torque_required[0]);

}




void PID_torque_2_dipole(PID_Inputs_struct *PID_Inputs){


    PID_Inputs->dipole_Moment[0] = ((PID_Inputs->B[1] * PID_Inputs->Torque_required[2]) - (PID_Inputs->B[2] * PID_Inputs->Torque_required[1])) /
                            ((PID_Inputs->B[0] * PID_Inputs->B[0]) + (PID_Inputs->B[1] * PID_Inputs->B[1]) + (PID_Inputs->B[2] * PID_Inputs->B[2]));

    PID_Inputs->dipole_Moment[1] = ((PID_Inputs->B[2] * PID_Inputs->Torque_required[0]) - (PID_Inputs->B[0] * PID_Inputs->Torque_required[2])) /
                            ((PID_Inputs->B[0] * PID_Inputs->B[0]) + (PID_Inputs->B[1] * PID_Inputs->B[1]) + (PID_Inputs->B[2] * PID_Inputs->B[2]));

    PID_Inputs->dipole_Moment[2] = ((PID_Inputs->B[0] * PID_Inputs->Torque_required[1]) - (PID_Inputs->B[1] * PID_Inputs->Torque_required[0])) /
                            ((PID_Inputs->B[0] * PID_Inputs->B[0]) + (PID_Inputs->B[1] * PID_Inputs->B[1]) + (PID_Inputs->B[2] * PID_Inputs->B[2]));

   printf(" PID_Inputs->dipole_Moment[0]: %f \n",PID_Inputs->dipole_Moment[0]);
   printf(" PID_Inputs->dipole_Moment[1]: %f \n",PID_Inputs->dipole_Moment[1]);
   printf(" PID_Inputs->dipole_Moment[2]: %f \n",PID_Inputs->dipole_Moment[2]);

}



void PID_dipole_2_current(PID_Inputs_struct *PID_Inputs){


    PID_Inputs->th_Current[0] = PID_Inputs->dipole_Moment[0] / (PID_Inputs->N_spires[0] * PID_Inputs->A_torquers[0]);

    PID_Inputs->th_Current[1] = PID_Inputs->dipole_Moment[1] / (PID_Inputs->N_spires[1] * PID_Inputs->A_torquers[1]);

    PID_Inputs->th_Current[2] = PID_Inputs->dipole_Moment[2] / (PID_Inputs->N_spires[2] * PID_Inputs->A_torquers[2]);

    printf(" PID_Inputs->th_Current[0]: %f \n",PID_Inputs->th_Current[0]);
    printf(" PID_Inputs->th_Current[1]: %f \n",PID_Inputs->th_Current[1]);
    printf(" PID_Inputs->th_Current[2]: %f \n",PID_Inputs->th_Current[2]);


}


void PID_current_2_DutyCycle(PID_Inputs_struct *PID_Inputs){

    PID_Inputs->th_Dutycycle[0] =  100*((PID_Inputs->th_Current[0] * PID_Inputs->torquer_Req_Ohm[0]) / PID_Inputs->torquer_Vdd[0]);

    PID_Inputs->th_Dutycycle[1] =  100*((PID_Inputs->th_Current[1] * PID_Inputs->torquer_Req_Ohm[1]) / PID_Inputs->torquer_Vdd[1]);

    PID_Inputs->th_Dutycycle[2] =  100*((PID_Inputs->th_Current[2] * PID_Inputs->torquer_Req_Ohm[2]) / PID_Inputs->torquer_Vdd[2]);

    printf(" PID_Inputs->th_Dutycycle[0]: %f \n",PID_Inputs->th_Dutycycle[0]);
    printf(" PID_Inputs->th_Dutycycle[1]: %f \n",PID_Inputs->th_Dutycycle[0]);
    printf(" PID_Inputs->th_Dutycycle[2]: %f \n",PID_Inputs->th_Dutycycle[0]);

}

void PID_INIT(PID_Inputs_struct *PID_Inputs){

  for (uint8_t i = 0; i < 3; i++) {

    PID_Inputs->N_spires[i] = N_spires[i];
    PID_Inputs->A_torquers[i] = A_torquers[i];
    PID_Inputs->torquer_Req_Ohm[i] = torquer_Req_Ohm[i];
    PID_Inputs->torquer_Vdd[i] = torquer_Vdd[i];
    PID_Inputs->P_Gain[i] = 1;
    PID_Inputs->D_Gain[i] = 2;
	PID_Inputs->angSpeed_Measured[i] = 0.1; //rad/s
	PID_Inputs->angSpeed_Desired[i] = 0.5; //rad/s
	PID_Inputs->B[i] = 0.05;
  }


}




// MAIN FUNCTION! Call it to do PID but read comment before
void PID_main(PID_Inputs_struct *PID_Inputs){

  // Update PID_Inputs_struct with gyro measurements and desired attitude and timestamp BEFORE CALLING this fuction

  PID_attitude_error_calculation(PID_Inputs);
  PID_attitude_Derivative_calculation(PID_Inputs);
  PID_angSpeed_error_2_torque(PID_Inputs);
  PID_torque_2_dipole(PID_Inputs);
  PID_dipole_2_current(PID_Inputs);
  PID_current_2_DutyCycle(PID_Inputs);

  for (uint8_t i = 0; i < 3; i++) {
  	PID_Inputs->angSpeed_Measured[i] = PID_Inputs->angSpeed_Measured[i] + PID_Inputs->angSpeed_Measured[i] / 10; //rad/s
  }

    // update of the ERROR buffer
  for (uint8_t i = 0; i < sizeof(PID_Inputs->d_AngSpeed_Err_dt); i++) {

    PID_Inputs->angSpeed_Error[i + 3] = PID_Inputs->angSpeed_Error[i];
    PID_Inputs->angSpeed_Error[i] = 0;

  }


}
