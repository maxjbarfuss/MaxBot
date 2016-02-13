#include <wiringPi.h>
#include <wiringSerial.h>

#include "RoboClawV4.h"

namespace MotionControl {

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(arg>>8),(uint8_t)arg

RoboClawV4::RoboClawV4(const char *device, int baud, bool doack)
{
    ack=doack;
    deviceId = serialOpen(device, baud);
}

RoboClawV4::~RoboClawV4()
{
    serialClose(deviceId);
}

bool RoboClawV4::write_n(uint8_t cnt, ... )
{
    uint8_t crc=0;

    va_list marker;
    va_start( marker, cnt );
    for(uint8_t index=0; index<cnt; index++)
    {
        uint8_t data = va_arg(marker, int);
        crc+=data;
        serialPutchar(deviceId, data);
    }
    va_end( marker );
    if(ack)
        serialPutchar(deviceId, ((crc&0x7F) | 0x80));
    else
        serialPutchar(deviceId, crc&0x7F);
    if(ack)
        if(serialGetchar(deviceId)==0xFF)
            return true;
    return false;
}

bool RoboClawV4::ForwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3,address,M1FORWARD,speed);
}

bool RoboClawV4::BackwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3,address,M1BACKWARD,speed);
}

bool RoboClawV4::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3,address,SETMINMB,voltage);
}

bool RoboClawV4::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3,address,SETMAXMB,voltage);
}

bool RoboClawV4::ForwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3,address,M2FORWARD,speed);
}

bool RoboClawV4::BackwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3,address,M2BACKWARD,speed);
}

bool RoboClawV4::ForwardBackwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3,address,M17BIT,speed);
}

bool RoboClawV4::ForwardBackwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3,address,M27BIT,speed);
}

bool RoboClawV4::ForwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3,address,MIXEDFORWARD,speed);
}

bool RoboClawV4::BackwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3,address,MIXEDBACKWARD,speed);
}

bool RoboClawV4::TurnRightMixed(uint8_t address, uint8_t speed)
{
    return write_n(3,address,MIXEDRIGHT,speed);
}

bool RoboClawV4::TurnLeftMixed(uint8_t address, uint8_t speed)
{
    return write_n(3,address,MIXEDLEFT,speed);
}

bool RoboClawV4::ForwardBackwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3,address,MIXEDFB,speed);
}

bool RoboClawV4::LeftRightMixed(uint8_t address, uint8_t speed)
{
    return write_n(3,address,MIXEDLR,speed);
}

bool RoboClawV4::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
    uint8_t crc;
    serialPutchar(deviceId, address);
    crc=address;
    serialPutchar(deviceId, cmd);
    crc+=cmd;

    //send data with crc
    va_list marker;
    va_start( marker, cmd );     /* Initialize variable arguments. */
    for(uint8_t index=0; index<cnt; index++)
    {
        int *ptr = (int *)va_arg(marker, int);

        int value;
        uint8_t data = serialGetchar(deviceId);
        crc+=data;
        value=(int)data<<24;

        data = serialGetchar(deviceId);
        crc+=data;
        value|=(int)data<<16;

        data = serialGetchar(deviceId);
        crc+=data;
        value|=(int)data<<8;

        data = serialGetchar(deviceId);
        crc+=data;
        value|=(int)data;

        *ptr = value;
    }
    va_end( marker );              /* Reset variable arguments.      */

    uint8_t data = serialGetchar(deviceId);

    return ((crc&0x7F)==data);
}

int RoboClawV4::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status,bool *valid)
{
    uint8_t crc;
    serialPutchar(deviceId, address);
    crc=address;
    serialPutchar(deviceId, cmd);
    crc+=cmd;

    int value;
    uint8_t data = serialGetchar(deviceId);
    crc+=data;
    value=(int)data<<24;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(int)data<<16;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(int)data<<8;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(int)data;

    data = serialGetchar(deviceId);
    crc+=data;
    if(status)
        *status = data;

    data = serialGetchar(deviceId);
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

int RoboClawV4::ReadEncM1(uint8_t address, uint8_t *status,bool *valid)
{
    return Read4_1(address,GETM1ENC,status,valid);
}

int RoboClawV4::ReadEncM2(uint8_t address, uint8_t *status,bool *valid)
{
    return Read4_1(address,GETM2ENC,status,valid);
}

int RoboClawV4::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid)
{
    return Read4_1(address,GETM1SPEED,status,valid);
}

int RoboClawV4::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid)
{
    return Read4_1(address,GETM2SPEED,status,valid);
}

bool RoboClawV4::ResetEncoders(uint8_t address)
{
    return write_n(2,address,RESETENC);
}

bool RoboClawV4::ReadVersion(uint8_t address,char *version)
{
    uint8_t crc;
    serialPutchar(deviceId, address);
    crc=address;
    serialPutchar(deviceId, GETVERSION);
    crc+=GETVERSION;

    for(uint8_t i=0; i<32; i++)
    {
        version[i]=serialGetchar(deviceId);
        crc+=version[i];
        if(version[i]==0)
        {
            if((crc&0x7F)==serialGetchar(deviceId))
                return true;
            else
                return false;
        }
    }
    return false;
}

uint16_t RoboClawV4::Read2(uint8_t address,uint8_t cmd,bool *valid)
{
    uint8_t crc;
    serialPutchar(deviceId, address);
    crc=address;
    serialPutchar(deviceId, cmd);
    crc+=cmd;

    uint16_t value;
    uint8_t data = serialGetchar(deviceId);
    crc+=data;
    value=(uint16_t)data<<8;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(uint16_t)data;

    data = serialGetchar(deviceId);
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

int RoboClawV4::Read4(uint8_t address, uint8_t cmd, bool *valid)
{
    uint8_t crc;
    serialPutchar(deviceId, address);
    crc=address;
    serialPutchar(deviceId, cmd);
    crc+=cmd;

    int value;
    uint8_t data = serialGetchar(deviceId);
    crc+=data;
    value=(int)data<<24;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(int)data<<16;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(int)data<<8;

    data = serialGetchar(deviceId);
    crc+=data;
    value|=(int)data;

    data = serialGetchar(deviceId);
    if(valid)
        *valid = ((crc&0x7F)==data);

    return value;
}

uint16_t RoboClawV4::ReadMainBatteryVoltage(uint8_t address,bool *valid)
{
    return Read2(address,GETMBATT,valid);
}

uint16_t RoboClawV4::ReadLogicBattVoltage(uint8_t address,bool *valid)
{
    return Read2(address,GETLBATT,valid);
}

bool RoboClawV4::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3,address,SETMINLB,voltage);
}

bool RoboClawV4::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3,address,SETMAXLB,voltage);
}

bool RoboClawV4::SetM1VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, int qpps)
{
    int kd = kd_fp*65536;
    int kp = kp_fp*65536;
    int ki = ki_fp*65536;
    return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool RoboClawV4::SetM2VelocityPID(uint8_t address, float kd_fp, float kp_fp, float ki_fp, int qpps)
{
    int kd = kd_fp*65536;
    int kp = kp_fp*65536;
    int ki = ki_fp*65536;
    return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

int RoboClawV4::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid)
{
    return Read4_1(address,GETM1ISPEED,status,valid);
}

int RoboClawV4::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid)
{
    return Read4_1(address,GETM2ISPEED,status,valid);
}

bool RoboClawV4::DutyM1(uint8_t address, uint16_t duty)
{
    return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool RoboClawV4::DutyM2(uint8_t address, uint16_t duty)
{
    return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool RoboClawV4::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2)
{
    return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

bool RoboClawV4::SpeedM1(uint8_t address, int speed)
{
    return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

bool RoboClawV4::SpeedM2(uint8_t address, int speed)
{
    return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

bool RoboClawV4::SpeedM1M2(uint8_t address, int speed1, int speed2)
{
    return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClawV4::SpeedAccelM1(uint8_t address, int accel, int speed)
{
    return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

bool RoboClawV4::SpeedAccelM2(uint8_t address, int accel, int speed)
{
    return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
bool RoboClawV4::SpeedAccelM1M2(uint8_t address, int accel, int speed1, int speed2)
{
    return write_n(10,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClawV4::SpeedDistanceM1(uint8_t address, int speed, int distance, uint8_t flag)
{
    return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawV4::SpeedDistanceM2(uint8_t address, int speed, int distance, uint8_t flag)
{
    return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawV4::SpeedDistanceM1M2(uint8_t address, int speed1, int distance1, int speed2, int distance2, uint8_t flag)
{
    return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed2),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClawV4::SpeedAccelDistanceM1(uint8_t address, int accel, int speed, int distance, uint8_t flag)
{
    return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawV4::SpeedAccelDistanceM2(uint8_t address, int accel, int speed, int distance, uint8_t flag)
{
    return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClawV4::SpeedAccelDistanceM1M2(uint8_t address, int accel, int speed1, int distance1, int speed2, int distance2, uint8_t flag)
{
    return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClawV4::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2)
{
    bool valid;
    uint16_t value = Read2(address,GETBUFFERS,&valid);
    if(valid)
    {
        depth1 = value>>8;
        depth2 = value;
    }
    return valid;
}

bool RoboClawV4::ReadCurrents(uint8_t address, uint16_t &current1, uint16_t &current2)
{
    bool valid;
    uint16_t value = Read4(address,GETCURRENTS,&valid);
    if(valid)
    {
        current1 = value>>16;
        current2 = value&0xFFFF;
    }
    return valid;
}

bool RoboClawV4::SpeedAccelM1M2_2(uint8_t address, int accel1, int speed1, int accel2, int speed2)
{
    return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

bool RoboClawV4::SpeedAccelDistanceM1M2_2(uint8_t address, int accel1, int speed1, int distance1, int accel2, int speed2, int distance2, uint8_t flag)
{
    return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClawV4::DutyAccelM1(uint8_t address, uint16_t duty, uint16_t accel)
{
    return write_n(6,address,M1DUTY,SetWORDval(duty),SetWORDval(accel));
}

bool RoboClawV4::DutyAccelM2(uint8_t address, uint16_t duty, uint16_t accel)
{
    return write_n(6,address,M2DUTY,SetWORDval(duty),SetWORDval(accel));
}

bool RoboClawV4::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint16_t accel1, uint16_t duty2, uint16_t accel2)
{
    return write_n(10,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(accel1),SetWORDval(duty2),SetWORDval(accel2));
}

bool RoboClawV4::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,int &qpps)
{
    int Kp,Ki,Kd;
    bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
    Kp_fp = ((float)Kp)/65536;
    Ki_fp = ((float)Ki)/65536;
    Kd_fp = ((float)Kd)/65536;
    return valid;
}

bool RoboClawV4::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,int &qpps)
{
    int Kp,Ki,Kd;
    bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
    Kp_fp = ((float)Kp)/65536;
    Ki_fp = ((float)Ki)/65536;
    Kd_fp = ((float)Kd)/65536;
    return valid;
}

bool RoboClawV4::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max)
{
    return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClawV4::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max)
{
    return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClawV4::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max)
{
    uint16_t value;
    bool valid = read_n(1,address,GETMINMAXMAINVOLTAGES,&value);
    min=value>>16;
    max = value&0xFFFF;
    return valid;
}

bool RoboClawV4::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max)
{
    uint16_t value;
    bool valid = read_n(1,address,GETMINMAXLOGICVOLTAGES,&value);
    min=value>>16;
    max = value&0xFFFF;
    return valid;
}

bool RoboClawV4::SetM1PositionPID(uint8_t address,float kd_fp,float kp_fp,float ki_fp,float kiMax_fp,int deadzone,int min,int max)
{
    int kd=kd_fp*1024;
    int kp=kp_fp*1024;
    int ki=ki_fp*1024;
    int kiMax=kiMax_fp*1024;
    return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClawV4::SetM2PositionPID(uint8_t address,float kd_fp,float kp_fp,float ki_fp,float kiMax_fp,int deadzone,int min,int max)
{
    int kd=kd_fp*1024;
    int kp=kp_fp*1024;
    int ki=ki_fp*1024;
    int kiMax=kiMax_fp*1024;
    return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClawV4::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,int &DeadZone,int &Min,int &Max)
{
    int Kp,Ki,Kd,KiMax;
    bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
    Kp_fp = ((float)Kp)/1024;
    Ki_fp = ((float)Ki)/1024;
    Kd_fp = ((float)Kd)/1024;
    KiMax = ((float)KiMax_fp)/1024;
    return valid;
}

bool RoboClawV4::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,float &KiMax_fp,int &DeadZone,int &Min,int &Max)
{
    int Kp,Ki,Kd,KiMax;
    bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
    Kp_fp = ((float)Kp)/1024;
    Ki_fp = ((float)Ki)/1024;
    Kd_fp = ((float)Kd)/1024;
    KiMax = ((float)KiMax_fp)/1024;
    return valid;
}

bool RoboClawV4::SpeedAccelDeccelPositionM1(uint8_t address,int accel,int speed,int deccel,int position,uint8_t flag)
{
    return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClawV4::SpeedAccelDeccelPositionM2(uint8_t address,int accel,int speed,int deccel,int position,uint8_t flag)
{
    return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClawV4::SpeedAccelDeccelPositionM1M2(uint8_t address,int accel1,int speed1,int deccel1,int position1,int accel2,int speed2,int deccel2,int position2,uint8_t flag)
{
    return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool RoboClawV4::ReadTemp(uint8_t address, uint16_t &temp)
{
    bool valid;
    temp = Read2(address,GETTEMP,&valid);
    return valid;
}

uint16_t RoboClawV4::ReadError(uint8_t address,bool *valid)
{
    return Read2(address,GETERROR,valid);
}

bool RoboClawV4::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode)
{
    bool valid;
    uint16_t value = Read2(address,GETENCODERMODE,&valid);
    if(valid)
    {
        M1mode = value>>8;
        M2mode = value;
    }
    return valid;
}

bool RoboClawV4::SetM1EncoderMode(uint8_t address,uint8_t mode)
{
    return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool RoboClawV4::SetM2EncoderMode(uint8_t address,uint8_t mode)
{
    return write_n(3,address,SETM2ENCODERMODE,mode);
}

bool RoboClawV4::WriteNVM(uint8_t address)
{
    return write_n(2,address,WRITENVM);
}

};
