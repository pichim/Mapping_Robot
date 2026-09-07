#ifndef MR_CONFIG_H_
#define MR_CONFIG_H_

#include "PESBoardPinMap.h"

// SPI Communication with Raspberry Pi
#define MR_SPI_SLAVE_DMA_MOSI_PIN PC_3
#define MR_SPI_SLAVE_DMA_MISO_PIN PC_2
#define MR_SPI_SLAVE_DMA_SCK_PIN PB_10
#define MR_SPI_SLAVE_DMA_NSS_PIN PB_12

#define MR_SPI_SLAVE_DMA_THREAD_PRIORITY osPriorityHigh2
#define MR_SPI_SLAVE_DMA_THREAD_STACK_SIZE OS_STACK_SIZE

// MPU6500 IMU Configuration
#define MR_IMU_SDA_PIN PB_9 // would be PC_9 on new PES_Board
#define MR_IMU_SCL_PIN PB_8 // would be PA_8 on new PES_Board
#define MR_IMU_USE_ADDITIONAL_FILTERS true
#define MR_IMU_GYRO_FILTER_FREQUENCY_HZ 60.0f
#define MR_IMU_ACC_FILTER_FREQUENCY_HZ 60.0f
#define MR_IMU_NUM_RUNS_SKIP 1000 // Samples; nominally 1 second at the configured 1 kHz loop rate
#define MR_IMU_NUM_RUNS_FOR_AVERAGE 1000
#define MR_IMU_DO_USE_STATIC_ACC_CALIBRATION true // Legacy, unused; IMU.cpp holds fixed accelerometer calibration
#define MR_IMU_B_ACC {0.0f, 0.0f, 0.0f} // Legacy, unused
#define MR_IMU_KP (0.1592f * 2.0f * M_PIf)
#define MR_IMU_KI 0.0f

#define MR_MOTOR_ENABLE_PIN PB_ENABLE_DCMOTORS
#define MR_MOTOR_M1_PWM_PIN PB_PWM_M1
#define MR_MOTOR_M1_ENC_A_PIN PB_ENC_A_M1
#define MR_MOTOR_M1_ENC_B_PIN PB_ENC_B_M1
#define MR_MOTOR_M2_PWM_PIN PB_PWM_M2
#define MR_MOTOR_M2_ENC_A_PIN PB_ENC_A_M2
#define MR_MOTOR_M2_ENC_B_PIN PB_ENC_B_M2
#define MR_MOTOR_GEAR_RATIO 156.25f
#define MR_MOTOR_KN_RPM_PER_V (89.0f / 12.0f)
#define MR_MOTOR_VOLTAGE_MAX 12.0f
#define MR_MOTOR_COUNTS_PER_TURN 20.0f
#define MR_WHEEL_DIAMETER_M 0.0822f
#define MR_WHEEL_SPACING_M 0.1435f
#define MR_MOTOR_COMMAND_TIMEOUT_US 250000

// SPI Communication and Control Thread
#define MR_SPI_COM_CNTRL_THREAD_PERIOD_US 1000
#define MR_SPI_COM_CNTRL_THREAD_PRIORITY osPriorityNormal
#define MR_SPI_COM_CNTRL_THREAD_STACK_SIZE OS_STACK_SIZE

// UART for logging communication (stream to PC for the fast running threads)
#define MR_LOG_COM_UART_TX_PIN PC_10 // untested!
#define MR_LOG_COM_UART_RX_PIN PC_11

#endif /* MR_CONFIG_H_ */
