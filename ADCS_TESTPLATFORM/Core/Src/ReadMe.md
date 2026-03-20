# main.c

File main.c, within main(), there are all hardware initialization functions (GPIO, UART4, USART1, USART2, timers TIM1, TIM2, TIM3, ADC),

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

it creates RTOS objects (mutex, queues) and threads (First Check, IMU Task, OBC Communication Task, Control Algorithm Task),

```
FirstCheckTaskHandle = osThreadCreate(osThread(FirstCheckTask), NULL);
IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);
OBC_CommTaskHandle = osThreadCreate(osThread(OBC_CommTask), NULL);
ControlAlgorithmTaskHandle = osThreadCreate(osThread(ControlAlgorithmTask), NULL);
```

# MTi1.c

File MTi1.c contains functions used to initialize and read the Inertial Measurement Unit (IMU). When IMU_GROUND_CALIBRATION is defined, the estimated gyroscope bias is printed out. Gyroscope can only be calibrated when the system is stationary on the ground; it cannot be calibrated in orbit, as the system will be tumbling when it is turned on.
