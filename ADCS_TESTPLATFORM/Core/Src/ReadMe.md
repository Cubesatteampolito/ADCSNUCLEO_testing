# main.c

File main.c, within main(), contains all hardware initialization functions (GPIO, UART4, USART1, USART2, timers TIM1, TIM2, TIM3, ADC):

```
Hal_init()
SystemClock_Config()
MX_GPIO_Init()
MX_USART2_UART_Init();
MX_UART4_Init();
MX_USART1_UART_Init();
MX_TIM1_Init();
MX_TIM2_Init();
MX_TIM3_Init();
MX_ADC1_Init();
initDriver_UART();
```

It also creates RTOS objects (mutex, queues) and threads (First Check, IMU Task, OBC Communication Task, Control Algorithm Task):

```
MX_FREERTOS_Init()
```

Finally, it starts the RTOS scheduler, which takes control of the CPU, starts running tasks and switches between them. The while(1) loop in main() is empty because main() is not in charge, the scheduler is. Execution should never reach while(1) unless `osKernelStart()` fails or RTOS misconfigured. Using RTOS in place of a bare-metal loop allows real-time scheduling and task prioritization.

# freertos.c

File freertos.c contains the OS variable definitions

It also contains the freeRTOS initialization function MX_FREERTOS_Init() used to create mutexes and semaphores, 3 queues (IMU1, IMU2 and housekeeping) and 4 threads (first check task, IMU task, OBC communication task and Control task).

# RTOS_Tasks.c

File RTOS_Tasks.c contains the function definitions for all tasks.

## MTi1.c

File MTi1.c contains functions used to initialize and read the Inertial Measurement Unit (IMU). When IMU_GROUND_CALIBRATION is defined, the estimated gyroscope bias is printed out. Gyroscope can only be calibrated when the system is stationary on the ground; it cannot be calibrated in orbit, as the system will be tumbling when it is turned on.

# messages.h

This file contains the structure of messages that are exchanged with the OBC.
