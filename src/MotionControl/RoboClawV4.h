#pragma once

#include <MotionControl/IRoboClaw.h>

namespace MotionControl {

class RoboClawV4 : public IRoboClaw
{
private:
    int deviceId;
    bool ack;
    bool write_n(uint8_t byte,...);
    bool read_n(uint8_t byte,uint8_t address,uint8_t cmd,...);
    uint32_t Read4_1(uint8_t address,uint8_t cmd,uint8_t *status,bool *valid);
    uint32_t Read4(uint8_t address,uint8_t cmd,bool *valid);
    uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid);
public:
    RoboClawV4(const char *device, int baud, bool doack=false);	//ack option only available on 3.1.8 and newer firmware
    ~RoboClawV4();
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
    virtual bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,float &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max) override;
    virtual bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,float &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max) override;
    virtual bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) override;
    virtual bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) override;
    virtual bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag) override;
    virtual bool ReadTemp(uint8_t address, uint16_t &temp) override;
    virtual uint16_t ReadError(uint8_t address,bool *valid=NULL) override;
    virtual bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode) override;
    virtual bool SetM1EncoderMode(uint8_t address,uint8_t mode) override;
    virtual bool SetM2EncoderMode(uint8_t address,uint8_t mode) override;
    virtual bool WriteNVM(uint8_t address) override;
};

};
