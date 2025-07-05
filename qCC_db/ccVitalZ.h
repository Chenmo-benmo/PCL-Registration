#pragma once
//添加 Chenmobenmo 241002
#include <iostream>
#include "CCConst.h"

namespace ccVitalZ
{
//记录三个点云重心的高度
extern float ZI;
extern float ZL;
extern float ZR;

//记录三个点云在各自坐标的重心坐标
extern float LX; extern float LY; extern float LZ;
extern float IX; extern float IY; extern float IZ;
extern float RX; extern float RY; extern float RZ;

//记录三个点云在各自坐标的重心所在XZ切面的最左端和最右端
extern float L_MIN_X; extern float L_MAX_X;
extern float I_MIN_X; extern float I_MAX_X;
extern float R_MIN_X; extern float R_MAX_X;

//记录三个点云的平均高度
extern float LH;
extern float IH;
extern float RH;

//得出三个点云在各自坐标的标记点
extern float L_VITAL_X; extern float L_VITAL_Y; extern float L_VITAL_Z;
extern float I_VITAL_X; extern float I_VITAL_Y; extern float I_VITAL_Z;
extern float R_VITAL_X; extern float R_VITAL_Y; extern float R_VITAL_Z;

//***************************************************************************
//***************************************************************************
//***************************************************************************

//获取数据
inline extern void Set_Vital_Z(float vital_Z, std::string name);

inline extern void Set_BaryXYZ(float x, float y, float z, std::string name);

inline extern void Set_MMX(float min, float max, std::string name);

inline extern void Set_High(float h, std::string name);

//***************************************************************************
//***************************************************************************
//***************************************************************************

//计算
inline extern void IWork(int rad);

inline extern void LWork(int rad);

inline extern void RWork(int rad);

inline extern void AllWork(int rad);
};


