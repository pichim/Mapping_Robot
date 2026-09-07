#ifndef AMR_CONFIG_H_
#define AMR_CONFIG_H_

// SPI Communication with Raspberry Pi
#define AMR_SPI_SLAVE_DMA_MOSI_PIN PC_3
#define AMR_SPI_SLAVE_DMA_MISO_PIN PC_2
#define AMR_SPI_SLAVE_DMA_SCK_PIN PB_10
#define AMR_SPI_SLAVE_DMA_NSS_PIN PB_12

#define AMR_SPI_SLAVE_DMA_THREAD_PRIORITY osPriorityHigh2
#define AMR_SPI_SLAVE_DMA_THREAD_STACK_SIZE OS_STACK_SIZE

// MPU6500 IMU Configuration
#define AMR_IMU_SDA_PIN PB_9 // would be PC_9 on new PES_Board
#define AMR_IMU_SCL_PIN PB_8 // would be PA_8 on new PES_Board
#define AMR_IMU_USE_ADDITIONAL_FILTERS true
#define AMR_IMU_GYRO_FILTER_FREQUENCY_HZ 60.0f
#define AMR_IMU_ACC_FILTER_FREQUENCY_HZ 60.0f
#define AMR_IMU_NUM_RUNS_SKIP 1000 // dont make this shorter than 1000 milliseconds, the openlager needs 1 second to start up (if used for logging)
#define AMR_IMU_NUM_RUNS_FOR_AVERAGE 3000
#define AMR_IMU_DO_USE_STATIC_ACC_CALIBRATION true // if this is true then averaged acc gets overwritten by AMR_IMU_B_ACC
#define AMR_IMU_B_ACC {0.0f, 0.0f, 0.0f}
#define AMR_IMU_KP (0.1592f * 2.0f * M_PIf)
#define AMR_IMU_KI 0.0f
#define AMR_DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define AMR_RAD_TO_DEG (180.0f / 3.14159265358979323846f)

// PES Board Servo Connections
#define AMR_SERVO_D0_PIN PB_2
#define AMR_SERVO_D1_PIN PC_8
#define AMR_SERVO_D2_PIN PC_6
#define AMR_SERVO_PWM_PERIOD_US 3000 // 20 ms period for standard hobby servos only temporary!
// #define AMR_SERVO_angle_range_rad 2.141519f

#define AMR_SERVO1_angle_range_grad 116.35f
#define AMR_SERVO2_angle_range_grad 115.55f
#define AMR_SERVO3_angle_range_grad 114.4f

// SPI Communication and Control Thread
#define AMR_SPI_COM_CNTRL_THREAD_PERIOD_US 1000
#define AMR_SPI_COM_CNTRL_THREAD_PRIORITY osPriorityNormal
#define AMR_SPI_COM_CNTRL_THREAD_STACK_SIZE OS_STACK_SIZE

// UART for logging communication (stream to PC for the fast running threads)

#define AMR_LOG_COM_UART_TX_PIN PA_9
#define AMR_LOG_COM_UART_RX_PIN PA_10

#define AMR_USER_BUTTON PB_1


#endif /* AMR_CONFIG_H_ */
