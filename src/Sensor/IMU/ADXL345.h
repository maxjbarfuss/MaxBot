#pragma once

#include <Dense>

#include <IO/I2CFactory.hpp>
#include <IO/II2C.h>
#include <Sensor/ISensor.h>

namespace Sensor {

//=========================================================================
//  I2C ADDRESS/BITS
#define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
//=========================================================================
//  REGISTERS
#define ADXL345_REG_DEVID               (0x00)    // Device ID
#define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
#define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
#define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
#define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
#define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
#define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
#define ADXL345_REG_DATAX               (0x32)    // X-axis data 0
#define ADXL345_REG_DATAY               (0x34)    // Y-axis data 0
#define ADXL345_REG_DATAZ               (0x36)    // Z-axis data 0
//=========================================================================

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0x0F,   // 1600Hz Bandwidth
  ADXL345_DATARATE_1600_HZ    = 0x0E,   //  800Hz Bandwidth
  ADXL345_DATARATE_800_HZ     = 0x0D,   //  400Hz Bandwidth
  ADXL345_DATARATE_400_HZ     = 0x0C,   //  200Hz Bandwidth
  ADXL345_DATARATE_200_HZ     = 0x0B,   //  100Hz Bandwidth
  ADXL345_DATARATE_100_HZ     = 0x0A,   //   50Hz Bandwidth
  ADXL345_DATARATE_50_HZ      = 0x09,   //   25Hz Bandwidth
  ADXL345_DATARATE_25_HZ      = 0x08,   // 12.5Hz Bandwidth
  ADXL345_DATARATE_12_5HZ     = 0x07,   // 6.25Hz Bandwidth
  ADXL345_DATARATE_6_25_HZ    = 0x06,   // 3.13Hz Bandwidth
  ADXL345_DATARATE_3_13_HZ    = 0x05,   // 1.56Hz Bandwidth
  ADXL345_DATARATE_1_56_HZ    = 0x04,   //  .78Hz Bandwidth
  ADXL345_DATARATE_0_78_HZ    = 0x03,   //  .39Hz Bandwidth
  ADXL345_DATARATE_0_39_HZ    = 0x02,   //   .2Hz Bandwidth
  ADXL345_DATARATE_0_20_HZ    = 0x01,   //   .1Hz Bandwidth
  ADXL345_DATARATE_0_10_HZ    = 0x00    //  .05Hz Bandwidth
} ADXL345DataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0x03,   // +/- 16g
  ADXL345_RANGE_8_G           = 0x02,   // +/- 8g
  ADXL345_RANGE_4_G           = 0x01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0x00    // +/- 2g (default value)
} ADXL345Range_t;

///****************************************************************************
/// ADXL345 - Accelerometer
///****************************************************************************
class ADXL345 : public ISensor<Eigen::Vector3d> {

private:
    std::unique_ptr<IO::II2C>   _i2c;
    Eigen::Vector3d             _average;
    double                      _scale;
private:
    void Start();
    void set_range(ADXL345Range_t range);
    void set_bandwidth(ADXL345DataRate_t bw);
public:
    ADXL345(IO::I2CFactory& i2CFactory, ADXL345DataRate_t rate, ADXL345Range_t range);
    virtual void StartCalibration();
    virtual void StepCalibration(int step);
    virtual void EndCalibration();
    Eigen::Vector3d virtual GetReading();
};

};
