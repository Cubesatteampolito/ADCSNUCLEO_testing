/* Configuration parameters */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

#define DEBUG_MSGS 1           // Enable/Disable debug messages via printf (1/0)


#define POWER_SUPPLY 12         // Volt
#define NUM_DRIVERS 5           // Number of drivers


#define NUM_ACTUATORS 5
#define NUM_TEMP_SENS 8


#define ln(x) log(x)
#define ADC_NUM_CHANNELS 16     // Number of ADC channels
#define Vref 3.3                // Volt

#define stack_size 768          // was 4096
#define stack_size1 2816        // was 16384
//reallocation of stack sizes to reduce memory usage

#endif /* INC_CONSTANTS_H_ */
