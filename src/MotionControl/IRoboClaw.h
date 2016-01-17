#pragma once

namespace MotionControl {

class IRoboClaw {
public:
    enum {M1FORWARD = 0,
			M1BACKWARD = 1,
			SETMINMB = 2,
			SETMAXMB = 3,
			M2FORWARD = 4,
			M2BACKWARD = 5,
			M17BIT = 6,
			M27BIT = 7,
			MIXEDFORWARD = 8,
			MIXEDBACKWARD = 9,
			MIXEDRIGHT = 10,
			MIXEDLEFT = 11,
			MIXEDFB = 12,
			MIXEDLR = 13,
			GETM1ENC = 16,
			GETM2ENC = 17,
			GETM1SPEED = 18,
			GETM2SPEED = 19,
			RESETENC = 20,
			GETVERSION = 21,
			SETM1ENCCOUNT = 22,
			SETM2ENCCOUNT = 23,
			GETMBATT = 24,
			GETLBATT = 25,
			SETMINLB = 26,
			SETMAXLB = 27,
			SETM1PID = 28,
			SETM2PID = 29,
			GETM1ISPEED = 30,
			GETM2ISPEED = 31,
			M1DUTY = 32,
			M2DUTY = 33,
			MIXEDDUTY = 34,
			M1SPEED = 35,
			M2SPEED = 36,
			MIXEDSPEED = 37,
			M1SPEEDACCEL = 38,
			M2SPEEDACCEL = 39,
			MIXEDSPEEDACCEL = 40,
			M1SPEEDDIST = 41,
			M2SPEEDDIST = 42,
			MIXEDSPEEDDIST = 43,
			M1SPEEDACCELDIST = 44,
			M2SPEEDACCELDIST = 45,
			MIXEDSPEEDACCELDIST = 46,
			GETBUFFERS = 47,
			GETPWMS = 48,
			GETCURRENTS = 49,
			MIXEDSPEED2ACCEL = 50,
			MIXEDSPEED2ACCELDIST = 51,
			M1DUTYACCEL = 52,
			M2DUTYACCEL = 53,
			MIXEDDUTYACCEL = 54,
			READM1PID = 55,
			READM2PID = 56,
			SETMAINVOLTAGES = 57,
			SETLOGICVOLTAGES = 58,
			GETMINMAXMAINVOLTAGES = 59,
			GETMINMAXLOGICVOLTAGES = 60,
			SETM1POSPID = 61,
			SETM2POSPID = 62,
			READM1POSPID = 63,
			READM2POSPID = 64,
			M1SPEEDACCELDECCELPOS = 65,
			M2SPEEDACCELDECCELPOS = 66,
			MIXEDSPEEDACCELDECCELPOS = 67,
			SETM1DEFAULTACCEL = 68,
			SETM2DEFAULTACCEL = 69,
			SETPINFUNCTIONS = 74,
			GETPINFUNCTIONS = 75,
			SETDEADBAND	= 76,
			GETDEADBAND	= 77,
			GETENCODERS = 78,
			GETISPEEDS = 79,
			RESTOREDEFAULTS = 80,
			GETTEMP = 82,
			GETTEMP2 = 83,	//Only valid on some models
			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94,
			READNVM = 95,	//Reloads values from Flash into Ram
			SETCONFIG = 98,
			GETCONFIG = 99,
			SETM1MAXCURRENT = 133,
			SETM2MAXCURRENT = 134,
			GETM1MAXCURRENT = 135,
			GETM2MAXCURRENT = 136,
			SETPWMMODE = 148,
			GETPWMMODE = 149,
			FLAGBOOTLOADER = 255};	//Only available via USB communications

    virtual bool ForwardM1(uint8_t address, uint8_t speed) = 0;
    virtual bool BackwardM1(uint8_t address, uint8_t speed) = 0;
    virtual bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage) = 0;
    virtual bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage) = 0;
    virtual bool ForwardM2(uint8_t address, uint8_t speed) = 0;
    virtual bool BackwardM2(uint8_t address, uint8_t speed) = 0;
    virtual bool ForwardBackwardM1(uint8_t address, uint8_t speed) = 0;
    virtual bool ForwardBackwardM2(uint8_t address, uint8_t speed) = 0;
    virtual bool ForwardMixed(uint8_t address, uint8_t speed) = 0;
    virtual bool BackwardMixed(uint8_t address, uint8_t speed) = 0;
    virtual bool TurnRightMixed(uint8_t address, uint8_t speed) = 0;
    virtual bool TurnLeftMixed(uint8_t address, uint8_t speed) = 0;
    virtual bool ForwardBackwardMixed(uint8_t address, uint8_t speed) = 0;
    virtual bool LeftRightMixed(uint8_t address, uint8_t speed) = 0;
    virtual uint32_t ReadEncM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) = 0;
    virtual uint32_t ReadEncM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) = 0;
    virtual uint32_t ReadSpeedM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) = 0;
    virtual uint32_t ReadSpeedM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL) = 0;
    virtual bool ResetEncoders(uint8_t address) = 0;
    virtual bool ReadVersion(uint8_t address,char *version) = 0;
    virtual uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL) = 0;
    virtual uint16_t ReadLogicBattVoltage(uint8_t address,bool *valid=NULL) = 0;
    virtual bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage) = 0;
    virtual bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage) = 0;
    virtual bool SetM1VelocityPID(uint8_t address, float Kd, float Kp, float Ki, uint32_t qpps) = 0;
    virtual bool SetM2VelocityPID(uint8_t address, float Kd, float Kp, float Ki, uint32_t qpps) = 0;
    virtual uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL) = 0;
    virtual uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL) = 0;
    virtual bool DutyM1(uint8_t address, uint16_t duty) = 0;
    virtual bool DutyM2(uint8_t address, uint16_t duty) = 0;
    virtual bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2) = 0;
    virtual bool SpeedM1(uint8_t address, uint32_t speed) = 0;
    virtual bool SpeedM2(uint8_t address, uint32_t speed) = 0;
    virtual bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2) = 0;
    virtual bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) = 0;
    virtual bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed) = 0;
    virtual bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2) = 0;
    virtual bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0) = 0;
    virtual bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0) = 0;
    virtual bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0) = 0;
    virtual bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0) = 0;
    virtual bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0) = 0;
    virtual bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0) = 0;
    virtual bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2) = 0;
    virtual bool ReadCurrents(uint8_t address, uint16_t &current1, uint16_t &current2) = 0;
    virtual bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2) = 0;
    virtual bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag=0) = 0;
    virtual bool DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel) = 0;
    virtual bool DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel) = 0;
    virtual bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2) = 0;
    virtual bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps) = 0;
    virtual bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps) = 0;
    virtual bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max) = 0;
    virtual bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max) = 0;
    virtual bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max) = 0;
    virtual bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max) = 0;
    virtual bool SetM1PositionPID(uint8_t address,float kd,float kp,float ki,float kiMax,uint32_t deadzone,uint32_t min,uint32_t max) = 0;
    virtual bool SetM2PositionPID(uint8_t address,float kd,float kp,float ki,float kiMax,uint32_t deadzone,uint32_t min,uint32_t max) = 0;
    virtual bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,float &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max) = 0;
    virtual bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,float &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max) = 0;
    virtual bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) = 0;
    virtual bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) = 0;
    virtual bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag) = 0;
    virtual bool ReadTemp(uint8_t address, uint16_t &temp) = 0;
    virtual uint16_t ReadError(uint8_t address,bool *valid=NULL) = 0;
    virtual bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode) = 0;
    virtual bool SetM1EncoderMode(uint8_t address,uint8_t mode) = 0;
    virtual bool SetM2EncoderMode(uint8_t address,uint8_t mode) = 0;
    virtual bool WriteNVM(uint8_t address) = 0;
};

};
