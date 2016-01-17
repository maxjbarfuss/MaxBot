#pragma once

#include <MotionControl/IRoboClaw.h>

namespace MotionControl {

#define _RC_VERSION 10 // software version of this library
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class RoboClawV5 : public IRoboClaw
{
private:
    int deviceId;
    uint16_t crc;
	uint32_t timeout;
	void crc_clear();
	uint16_t crc_get();
	void crc_update (uint8_t data);
	bool write_n(uint8_t byte,...);
	bool read_n(uint8_t byte,uint8_t address,uint8_t cmd,...);
	uint32_t Read4_1(uint8_t address,uint8_t cmd,uint8_t *status,bool *valid);
	uint32_t Read4(uint8_t address,uint8_t cmd,bool *valid);
	uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid);
	uint8_t Read1(uint8_t address,uint8_t cmd,bool *valid);
public:
    RoboClawV5(const char *device, int baud);
	~RoboClawV5();
	///V4 Compatible
	virtual bool ForwardM1(uint8_t address, uint8_t speed) override;
    virtual bool BackwardM1(uint8_t address, uint8_t speed) override;
    virtual bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage) override;
    virtual bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage) override;
    virtual bool ForwardM2(uint8_t address, uint8_t speed) override;
    virtual bool BackwardM2(uint8_t address, uint8_t speed) override;
    virtual bool ForwardBackwardM1(uint8_t address, uint8_t speed) override;
    virtual bool ForwardBackwardM2(uint8_t address, uint8_t speed) override;
    virtual bool ForwardMixed(uint8_t address, uint8_t speed) override;
    virtual bool BackwardMixed(uint8_t address, uint8_t speed) override;
    virtual bool TurnRightMixed(uint8_t address, uint8_t speed) override;
    virtual bool TurnLeftMixed(uint8_t address, uint8_t speed) override;
    virtual bool ForwardBackwardMixed(uint8_t address, uint8_t speed) override;
    virtual bool LeftRightMixed(uint8_t address, uint8_t speed) override;
    virtual uint32_t ReadEncM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) override;
    virtual uint32_t ReadEncM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) override;
    virtual uint32_t ReadSpeedM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) override;
    virtual uint32_t ReadSpeedM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) override;
    virtual bool ResetEncoders(uint8_t address) override;
    virtual bool ReadVersion(uint8_t address,char *version) override;
    virtual uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL) override;
    virtual uint16_t ReadLogicBattVoltage(uint8_t address,bool *valid=NULL) override;
    virtual bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage) override;
    virtual bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage) override;
    virtual bool SetM1VelocityPID(uint8_t address, float Kd, float Kp, float Ki, uint32_t qpps) override;
    virtual bool SetM2VelocityPID(uint8_t address, float Kd, float Kp, float Ki, uint32_t qpps) override;
    virtual uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL) override;
    virtual uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL) override;
    virtual bool DutyM1(uint8_t address, uint16_t duty) override;
    virtual bool DutyM2(uint8_t address, uint16_t duty) override;
    virtual bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2) override;
    virtual bool SpeedM1(uint8_t address, uint32_t speed) override;
    virtual bool SpeedM2(uint8_t address, uint32_t speed) override;
    virtual bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2) override;
    virtual bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) override;
    virtual bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed) override;
    virtual bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2) override;
    virtual bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0) override;
    virtual bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0) override;
    virtual bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0) override;
    virtual bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0) override;
    virtual bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0) override;
    virtual bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0) override;
    virtual bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2) override;
    virtual bool ReadCurrents(uint8_t address, uint16_t &current1, uint16_t &current2) override;
    virtual bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2) override;
    virtual bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag=0) override;
    virtual bool DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel) override;
    virtual bool DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel) override;
    virtual bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2) override;
    virtual bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps) override;
    virtual bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps) override;
    virtual bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max) override;
    virtual bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max) override;
    virtual bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max) override;
    virtual bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max) override;
    virtual bool SetM1PositionPID(uint8_t address,float kd,float kp,float ki,float kiMax,uint32_t deadzone,uint32_t min,uint32_t max) override;
    virtual bool SetM2PositionPID(uint8_t address,float kd,float kp,float ki,float kiMax,uint32_t deadzone,uint32_t min,uint32_t max) override;
    virtual bool ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max) override;
    virtual bool ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max) override;
    virtual bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) override;
    virtual bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) override;
    virtual bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag) override;
    virtual bool ReadTemp(uint8_t address, uint16_t &temp) override;
    virtual uint16_t ReadError(uint8_t address,bool *valid=NULL) override;
    virtual bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode) override;
    virtual bool SetM1EncoderMode(uint8_t address,uint8_t mode) override;
    virtual bool SetM2EncoderMode(uint8_t address,uint8_t mode) override;
    virtual bool WriteNVM(uint8_t address) override;
    /// V5
	virtual bool SetEncM1(uint8_t address, int32_t val);
	virtual bool SetEncM2(uint8_t address, int32_t val);
	virtual bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2);
	virtual bool SetM1DefaultAccel(uint8_t address, uint32_t accel);
	virtual bool SetM2DefaultAccel(uint8_t address, uint32_t accel);
	virtual bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
	virtual bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode);
	virtual bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max);
	virtual bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max);
	virtual bool ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2);
	virtual bool ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2);
	virtual bool RestoreDefaults(uint8_t address);
    virtual bool ReadTemp2(uint8_t address, uint16_t &temp);
	virtual bool ReadNVM(uint8_t address);
	virtual bool SetConfig(uint8_t address, uint16_t config);
	virtual bool GetConfig(uint8_t address, uint16_t &config);
	virtual bool SetM1MaxCurrent(uint8_t address,uint32_t max);
	virtual bool SetM2MaxCurrent(uint8_t address,uint32_t max);
	virtual bool ReadM1MaxCurrent(uint8_t address,uint32_t &max);
	virtual bool ReadM2MaxCurrent(uint8_t address,uint32_t &max);
	virtual bool SetPWMMode(uint8_t address, uint8_t mode);
	virtual bool GetPWMMode(uint8_t address, uint8_t &mode);
};

};
