#include "SPIComCntrl.h"

#include <cmath>

SPIComCntrl::SPIComCntrl()
    : RealTimeThread(
          MR_SPI_COM_CNTRL_THREAD_PERIOD_US, MR_SPI_COM_CNTRL_THREAD_PRIORITY, MR_SPI_COM_CNTRL_THREAD_STACK_SIZE)
    , m_SpiSlaveDMA(MR_SPI_SLAVE_DMA_MOSI_PIN,
                    MR_SPI_SLAVE_DMA_MISO_PIN,
                    MR_SPI_SLAVE_DMA_SCK_PIN,
                    MR_SPI_SLAVE_DMA_NSS_PIN,
                    MR_SPI_SLAVE_DMA_THREAD_PRIORITY,
                    MR_SPI_SLAVE_DMA_THREAD_STACK_SIZE)
    , m_Imu(MR_IMU_SDA_PIN, MR_IMU_SCL_PIN)
    , m_enable_motors(MR_MOTOR_ENABLE_PIN, 0)
    , m_Motor_M1(MR_MOTOR_M1_PWM_PIN,
                 MR_MOTOR_M1_ENC_A_PIN,
                 MR_MOTOR_M1_ENC_B_PIN,
                 MR_MOTOR_GEAR_RATIO,
                 MR_MOTOR_KN_RPM_PER_V,
                 MR_MOTOR_VOLTAGE_MAX,
                 MR_MOTOR_COUNTS_PER_TURN)
    , m_Motor_M2(MR_MOTOR_M2_PWM_PIN,
                 MR_MOTOR_M2_ENC_A_PIN,
                 MR_MOTOR_M2_ENC_B_PIN,
                 MR_MOTOR_GEAR_RATIO,
                 MR_MOTOR_KN_RPM_PER_V,
                 MR_MOTOR_VOLTAGE_MAX,
                 MR_MOTOR_COUNTS_PER_TURN)
    , m_SerialStream(MR_LOG_COM_UART_TX_PIN, MR_LOG_COM_UART_RX_PIN)
{
    const float wheel_radius = MR_WHEEL_DIAMETER_M / 2.0f;
    Cwheel2robot << wheel_radius / 2.0f, wheel_radius / 2.0f, wheel_radius / MR_WHEEL_SPACING_M,
        -wheel_radius / MR_WHEEL_SPACING_M;
    robot_speed_setpoint.setZero();
    robot_speed.setZero();
    wheel_speed_setpoint.setZero();
    wheel_speed.setZero();
    m_SpiSlaveDMA.setReplyData(m_reply_data, SPI_NUM_FLOATS);

    // Start SPI communication; guard failure
    if (!m_SpiSlaveDMA.start()) {
        printf("SPI start() failed — check wiring, pin mapping, or DMA state.\n");
        return;
    }

    m_spi_ready = true;
    printf("SPI Communication started. Waiting for master...\n");

    m_Timer.start();

    // NOTE: RealTimeThread::enable() must be called by the user after construction is complete
}

SPIComCntrl::~SPIComCntrl() = default;

void SPIComCntrl::executeTask()
{
    // Return early if SPI not ready
    if (!m_spi_ready) {
        return;
    }

    // Measure delta time
    const microseconds time_us = m_Timer.elapsed_time();
    const float dtime_us = duration_cast<microseconds>(time_us - m_time_previous_us).count();
    m_time_previous_us = time_us;

    // Check for new SPI data from master
    if (m_SpiSlaveDMA.hasNewData()) {
        m_spiData = m_SpiSlaveDMA.getSPIData();
        m_have_command = m_Imu.isCalibrated() && std::isfinite(m_spiData.data[0]) && std::isfinite(m_spiData.data[1]);
        if (m_have_command) {
            m_last_command_us = time_us;
        }
    }

    wheel_speed = {m_Motor_M1.getVelocity() * (2.0f * M_PIf),
                   m_Motor_M2.getVelocity() * (2.0f * M_PIf)};
    robot_speed = Cwheel2robot * wheel_speed;

    robot_speed_setpoint = {m_spiData.data[0], m_spiData.data[1]};
    if (!m_have_command || time_us - m_last_command_us >= microseconds{MR_MOTOR_COMMAND_TIMEOUT_US}) {
        m_have_command = false;
        robot_speed_setpoint.setZero();
        m_enable_motors = 0;
    }

    wheel_speed_setpoint = Cwheel2robot.inverse() * robot_speed_setpoint;
    if (!wheel_speed_setpoint.allFinite()) {
        m_have_command = false;
        robot_speed_setpoint.setZero();
        wheel_speed_setpoint.setZero();
        m_enable_motors = 0;
    }
    m_Motor_M1.setVelocity(wheel_speed_setpoint(0) / (2.0f * M_PIf));
    m_Motor_M2.setVelocity(wheel_speed_setpoint(1) / (2.0f * M_PIf));
    m_enable_motors = m_have_command ? 1 : 0;

    // Read IMU data
    m_ImuData = m_Imu.getImuData();
    if (!m_Imu.isCalibrated())
        return;

    // Prepare next reply
    m_reply_data[0] = robot_speed(0);
    m_reply_data[1] = robot_speed(1);
    m_reply_data[2] = m_ImuData.gyro.x();
    m_reply_data[3] = m_ImuData.gyro.y();
    m_reply_data[4] = m_ImuData.gyro.z();
    m_reply_data[5] = m_ImuData.acc.x();
    m_reply_data[6] = m_ImuData.acc.y();
    m_reply_data[7] = m_ImuData.acc.z();
    m_reply_data[8] = m_ImuData.rpy.x();
    m_reply_data[9] = m_ImuData.rpy.y();
    m_reply_data[10] = m_ImuData.rpy.z();
    m_SpiSlaveDMA.setReplyData(m_reply_data, 11);

    // Send data over serial stream
    if (m_SerialStream.startByteReceived()) {
        m_SerialStream.write(dtime_us);
        m_SerialStream.write(robot_speed_setpoint(0));
        m_SerialStream.write(robot_speed_setpoint(1));
        for (size_t index = 0; index < 11; ++index) {
            m_SerialStream.write(m_reply_data[index]);
        }
        m_SerialStream.send();
    }
}
