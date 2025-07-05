#include "ccVitalZ.h"

namespace ccVitalZ {
float ZI = 0.0f;
float ZL = 0.0f;
float ZR = 0.0f;

float LX = 0.0f; float LY = 0.0f; float LZ = 0.0f;
float IX = 0.0f; float IY = 0.0f; float IZ = 0.0f;
float RX = 0.0f; float RY = 0.0f; float RZ = 0.0f;

float L_MIN_X = 0.0f; float L_MAX_X = 0.0f;
float I_MIN_X = 0.0f; float I_MAX_X = 0.0f;
float R_MIN_X = 0.0f; float R_MAX_X = 0.0f;

float LH = 0.0f;
float IH = 0.0f;
float RH = 0.0f;

float L_VITAL_X = 0.0f; float L_VITAL_Y = 0.0f; float L_VITAL_Z = 0.0f;
float I_VITAL_X = 0.0f; float I_VITAL_Y = 0.0f; float I_VITAL_Z = 0.0f;
float R_VITAL_X = 0.0f; float R_VITAL_Y = 0.0f; float R_VITAL_Z = 0.0f;

//***************************************************************************
//***************************************************************************
//***************************************************************************

//获取数据
void Set_Vital_Z(float vital_Z, std::string name)
{
    if (name == "INFRONT")	ZI = vital_Z;
    if (name == "LEFT")		ZL = vital_Z;
    if (name == "RIGHT")	ZR = vital_Z;
}

void Set_BaryXYZ(float x, float y, float z, std::string name)
{
    if (name == "INFRONT") { IX = x; IY = y; IZ = z; }
    if (name == "LEFT") { LX = x; LY = y; LZ = z; }
    if (name == "RIGHT") { RX = x; RY = y; RZ = z; }
}

void Set_MMX(float min, float max, std::string name)
{
    if (name == "INFRONT") { I_MIN_X = min; I_MAX_X = max; }
    if (name == "LEFT") { L_MIN_X = min; L_MAX_X = max; }
    if (name == "RIGHT") { R_MIN_X = min; R_MAX_X = max; }
}

void Set_High(float h, std::string name)
{
    if (name == "INFRONT")	IH = h;
    if (name == "LEFT")		LH = h;
    if (name == "RIGHT")	RH = h;
}

//***************************************************************************
//***************************************************************************
//***************************************************************************

//计算
void IWork(int rad)
{
    float BigTriangle; //大三角边长
    float SmaTriangle; //小三角边长
    float BigTriCoorX; //大三角左下角X坐标
    float SmaTriCoorX; //小三角左下角X坐标
    float VitalX;	   //标记点X坐标
    float VitalY;      //标记点Y坐标
    float VitalZ;	   //标记点Z坐标

    double radians = (180.0 - abs(rad)) * (M_PI / 180.0);
    BigTriangle = (I_MAX_X + IH / tan(radians)) - (I_MIN_X - IH / tan(radians));
    SmaTriangle = BigTriangle - 2.0 * ZI / tan(radians) - ZL / sin(radians) - ZR / sin(radians);

    BigTriCoorX = I_MIN_X - IH / tan(radians);
    SmaTriCoorX = BigTriCoorX + ZI / tan(radians) + ZR / sin(radians);

    VitalX = SmaTriCoorX + SmaTriangle / 2.0;
    VitalY = IY;
    VitalZ = ZI + (SmaTriangle / 2.0) / tan(radians);

    double radian = rad * (M_PI / 180.0);
    I_VITAL_X = VitalX * cos(radian) - VitalZ * sin(radian);
    I_VITAL_Y = VitalY;
    I_VITAL_Z = VitalX * sin(radian) + VitalZ * cos(radian);
}

void LWork(int rad)
{
    float BigTriangle; //大三角边长
    float SmaTriangle; //小三角边长
    float BigTriCoorX; //大三角左下角X坐标
    float SmaTriCoorX; //小三角左下角X坐标
    float VitalX;	   //标记点X坐标
    float VitalY;      //标记点Y坐标
    float VitalZ;	   //标记点Z坐标

    double radians = (180.0 - abs(rad)) * (M_PI / 180.0);
    BigTriangle = (L_MAX_X + LH / tan(radians)) - (L_MIN_X - LH / tan(radians));
    SmaTriangle = BigTriangle - 2.0 * ZL / tan(radians) - ZI / sin(radians) - ZR / sin(radians);

    BigTriCoorX = L_MIN_X - LH / tan(radians);
    SmaTriCoorX = BigTriCoorX + ZL / tan(radians) + ZI / sin(radians);

    VitalX = SmaTriCoorX + SmaTriangle / 2.0;
    VitalY = LY;
    VitalZ = ZL + (SmaTriangle / 2.0) / tan(radians);

    double radian = rad * (M_PI / 180.0);
    L_VITAL_X = VitalX * cos(radian) - VitalZ * sin(radian);
    L_VITAL_Y = VitalY;
    L_VITAL_Z = VitalX * sin(radian) + VitalZ * cos(radian);
}

void RWork(int rad)
{
    float BigTriangle; //大三角边长
    float SmaTriangle; //小三角边长
    float BigTriCoorX; //大三角左下角X坐标
    float SmaTriCoorX; //小三角左下角X坐标
    float VitalX;	   //标记点X坐标
    float VitalY;      //标记点Y坐标
    float VitalZ;	   //标记点Z坐标

    double radians = (180.0 - abs(rad)) * (M_PI / 180.0);
    BigTriangle = (R_MAX_X + RH / tan(radians)) - (R_MIN_X - RH / tan(radians));
    SmaTriangle = BigTriangle - 2.0 * ZR / tan(radians) - ZL / sin(radians) - ZI / sin(radians);

    BigTriCoorX = R_MIN_X - RH / tan(radians);
    SmaTriCoorX = BigTriCoorX + ZR / tan(radians) + ZL / sin(radians);

    VitalX = SmaTriCoorX + SmaTriangle / 2.0;
    VitalY = RY;
    VitalZ = ZR + (SmaTriangle / 2.0) / tan(radians);

    double radian = rad * (M_PI / 180.0);
    R_VITAL_X = VitalX * cos(radian) - VitalZ * sin(radian);
    R_VITAL_Y = VitalY;
    R_VITAL_Z = VitalX * sin(radian) + VitalZ * cos(radian);
}

void AllWork(int rad)
{
    IWork(rad);
    LWork(rad);
    RWork(rad);
}
}
