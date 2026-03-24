/* Readings from IMU */ 
void IMU_Task(void const * argument); 
/* Communication with OBC - by now simulated with py program */ 
void OBC_Comm_Task(void const * argument); 
/* Attitude control logic */ 
void Control_Algorithm_Task(void const * argument); 
/* Current & Temperature monitors */ 
void Check_current_temp(void const * argument);