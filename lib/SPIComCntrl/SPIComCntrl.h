#ifndef SPI_COM_CNTRL_H_
#define SPI_COM_CNTRL_H_

#include "DCMotor.h"
#include "IMU.h"
#include "RealTimeThread.h"
#include "SPISlaveDMA.h"
#include "SerialStream.h"
#include "config.h"
#include "mbed.h"

using namespace std::chrono;

class SPIComCntrl : public RealTimeThread
{
public:
    explicit SPIComCntrl();
    virtual ~SPIComCntrl();

private:
    SpiData m_spiData;
    SpiSlaveDMA m_SpiSlaveDMA;

    ImuData m_ImuData;
    IMU m_Imu;

    DigitalOut m_enable_motors;
    DCMotor m_Motor_M1;
    DCMotor m_Motor_M2;

    Eigen::Matrix2f Cwheel2robot;
    Eigen::Vector2f robot_speed_setpoint;
    Eigen::Vector2f wheel_speed_setpoint;
    Eigen::Vector2f robot_speed;
    Eigen::Vector2f wheel_speed;

    SerialStream m_SerialStream;
    Timer m_Timer;
    microseconds m_time_previous_us{0};

    microseconds m_last_command_us{0};
    bool m_have_command{false};
    float m_reply_data[SPI_NUM_FLOATS]{};

    bool m_spi_ready{false};

    void executeTask() override;
};
#endif /* SPI_COM_CNTRL_H_ */
