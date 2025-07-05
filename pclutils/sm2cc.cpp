//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCL                        #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         COPYRIGHT: Luca Penasa                         #
//#                                                                        #
//##########################################################################
//
#include "sm2cc.h"

//Local
#include "PCLConv.h"
#include "my_point_types.h"
#include <iostream> //添加 Chenmobenmo 241107
#include <string_view> //添加 Chenmobenmo 241107
//PCL
#include <pcl/common/io.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccVitalZ.h>//添加 Chenmobenmo 241002

//PCL V1.6 or older
#ifdef PCL_VER_1_6_OR_OLDER

#include <sensor_msgs/PointField.h>
typedef sensor_msgs::PointField PCLScalarField;

#else //Version 1.7 or newer

#include <pcl/PCLPointField.h>
typedef pcl::PCLPointField PCLScalarField;

#endif

//system
#include <assert.h>

//添加 Chenmobenmo 241106
#include "ccViewer/ccviewer.h"

size_t GetNumberOfPoints(PCLCloud::Ptr sm_cloud)
{
	//用width*height表示点云中点的总数 Chenmobenmo 240926
	return static_cast<size_t>(sm_cloud ? sm_cloud->width * sm_cloud->height : 0);
}

bool ExistField(PCLCloud::Ptr sm_cloud, std::string name)
{
	if (sm_cloud)
		for (std::vector< PCLScalarField >::const_iterator it = sm_cloud->fields.begin(); it != sm_cloud->fields.end(); ++it)
			if (it->name == name)
				return true;

	return false;
}

sm2ccConverter::sm2ccConverter(PCLCloud::Ptr sm_cloud)
	: m_sm_cloud(sm_cloud)
{
	assert(sm_cloud);
}

ccPointCloud* sm2ccConverter::getCloud(bool cc/*添加 Chenmobenmo 240929*/, 
									   int rad/*添加 Chenmobenmo 240930*/, 
									   std::string name/*添加 Chenmobenmo 241001*/)
{
	//判断点云文件是否为空 Chenmobenmo 240926
	if (!m_sm_cloud)
	{
		assert(false);
		return 0;
	}
	
	//get the fields list
	//获取字段，字段通常包含xyz、颜色、法线 Chenmobenmo 240926
	std::list<std::string> fields;
	for (std::vector< PCLScalarField >::const_iterator it = m_sm_cloud->fields.begin(); it != m_sm_cloud->fields.end(); ++it)
	{
		if (it->name != "_") //PCL padding fields
			fields.push_back(it->name);
	}

	//begin with checks and conversions
	//be sure we have x, y, and z fields
	//判断字段是否包含xyz Chenmobenmo 240926
	if (!ExistField(m_sm_cloud,"x") || !ExistField(m_sm_cloud,"y") || !ExistField(m_sm_cloud,"z"))
		return 0;

	//create cloud
	//创建cloud并将点云坐标保存进protected类型cloud.m_points中 Chenmobenmo 240927
	ccPointCloud* cloud = new ccPointCloud();
	size_t expectedPointCount = GetNumberOfPoints(m_sm_cloud);	//获取点云文件里点的总数 Chenmobenmo 240926
	if (expectedPointCount != 0)
	{
		//push points inside
		//获取xyz Chenmobenmo 240926
		if (!addXYZ(cloud))
		{
			delete cloud;
			return 0;
		}
	}

	//添加 Chenmobenmo 241020
	//滤波器
	//运行有点慢，也许有更好的改进方法
	ccViewer& ccv = ccViewer::Get();
	if (ccv.ui.confirmed_chb->isChecked())
	{
		for (int i = 0; i < ccv.ui.interations_spb->text().toInt(); i++)
			filter(cloud, ccv.ui.quality_spb->text().toFloat() / 100.0, ccv.ui.searching_dsb->text().toFloat());
	}

	//获取移动前长宽 Chenmobenmo 241107
	Get_MM_XZ(cloud);

	//添加 X 轴对齐，但是没有任何作用 Chenmobenmo 241107
	if (ccv.ui.Xalign_chb->isChecked())
		Xalign(cloud);

	//添加 Chenmobenmo 241107
	{
		if (name == "LEFT")
		{
			ccv.ui.Left_XRatio_led->setText(QString::number(cloud->min_x) + " , " + QString::number(cloud->max_x));
			ccv.ui.Left_YRatio_led->setText(QString::number(cloud->min_y) + " , " + QString::number(cloud->max_y));
			ccv.ui.Left_ZRatio_led->setText(QString::number(cloud->min_z) + " , " + QString::number(cloud->max_z));
		}	
		if (name == "INFRONT")
		{
			ccv.ui.Infront_XRatio_led->setText(QString::number(cloud->min_x) + " , " + QString::number(cloud->max_x));
			ccv.ui.Infront_YRatio_led->setText(QString::number(cloud->min_y) + " , " + QString::number(cloud->max_y));
			ccv.ui.Infront_ZRatio_led->setText(QString::number(cloud->min_z) + " , " + QString::number(cloud->max_z));
		}
		if (name == "RIGHT")
		{
			ccv.ui.Right_XRatio_led->setText(QString::number(cloud->min_x) + " , " + QString::number(cloud->max_x));
			ccv.ui.Right_YRatio_led->setText(QString::number(cloud->min_y) + " , " + QString::number(cloud->max_y));
			ccv.ui.Right_ZRatio_led->setText(QString::number(cloud->min_z) + " , " + QString::number(cloud->max_z));
		}
	}		

	//添加 Chenmobenmo 240929
	//旋转并赋值ccVitalZ Chenmobenmo 241002
	if (cc == true)
	{
		if (ccv.ui.use_Zretract_chb->isChecked() && ccv.ui.Zretract_dsb->text().toFloat() != 0) //点云向 Z 轴内向缩进
			translate(cloud, -1.0 * ccv.ui.Zretract_dsb->text().toFloat()); //点云貌似是以Z = -3.5作为底面，向上平移 //微调 Chenmobenmo 241003
		revolve(cloud, rad, name);
		translate(cloud);	//按照标记点平移
	}
	else //添加 Chenmobenmo 241002
		Get_barycenter_point(cloud, rad, name); //设置ccVitalZ Chenmobenmo 241002

	//获取移动后长宽 Chenmobenmo 241003
	Get_MM_XZ(cloud);

	//添加 Y 轴对齐 Chenmobenmo 241106
	if (ccv.ui.Yalign_chb->isChecked())
		Yalign(cloud);

	//remove x,y,z fields from the vector of field names
	//移除所有"x""y""z"字符 Chenmobenmo 240927
	fields.remove("x");
	fields.remove("y");
	fields.remove("z");

	//do we have normals?
	//判断是否有字段记录法线，有则移除"normal_x""normal_y""normal_z"字符串 Chenmobenmo 240927
	if (ExistField(m_sm_cloud,"normal_x") || ExistField(m_sm_cloud,"normal_y") || ExistField(m_sm_cloud,"normal_z"))
	{
		addNormals(cloud);
		
		//remove the corresponding fields
		fields.remove("normal_x");
		fields.remove("normal_y");
		fields.remove("normal_z");
	}

	//The same for colors
	//颜色也一样的操作 Chenmobenmo 240927
	if (ExistField(m_sm_cloud, "rgb"))
	{
		addRGB(cloud);

		//remove the corresponding field
		fields.remove("rgb");
	}
	//The same for colors
	//颜色的rgba类型 Chenmobenmo 240927
	else if (ExistField(m_sm_cloud, "rgba"))
	{
		addRGB(cloud);

		//remove the corresponding field
		fields.remove("rgba");
	}

	//All the remaining fields will be stored as scalar fields
	//遍历并找到所有剩余的字段名，并将对应的数据作为标量添加到cloud中，通常是温度、强度、标签等 Chenmobenmo 240927
	for (std::list<std::string>::const_iterator name = fields.begin(); name != fields.end(); ++name)
	{
		addScalarField(cloud, *name);
	}

	return cloud;
}

bool sm2ccConverter::addXYZ(ccPointCloud *cloud)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	size_t pointCount = GetNumberOfPoints(m_sm_cloud);

	//预留加载空间 Chenmobenmo 240926
	if (!cloud->reserve(static_cast<unsigned>(pointCount)))
		return false;

	//add xyz to the given cloud taking xyz infos from the sm cloud
	//从pcl::PointCloud2转换到pcl::PointCloud<pcl::PointXYZ>，m_sm_cloud依旧指向点云数据 Chenmobenmo 240926
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	FROM_PCL_CLOUD(*m_sm_cloud, *pcl_cloud);

	//loop
	//遍历点云 Chenmobenmo 240926
	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3 P(pcl_cloud->at(i).x,
					pcl_cloud->at(i).y,
					pcl_cloud->at(i).z);

		//将坐标加载进cloud中 Chenmobenmo 240927
		cloud->addPoint(P);
	}

	return true;
}

//添加 Chenmobenmo 241020
void sm2ccConverter::filter(ccPointCloud* cloud, float probability/*删除周围点数百分比*/, float gridLength/*网格大小*/)
{
	cloud->filter(probability, gridLength);
}

//添加 Chenmobenmo 240929
//将中心点移动到原点
//添加 Chenmobenmo 240930
void sm2ccConverter::translate(ccPointCloud* cloud, float num)
{
	if (num == 0.0)
	{
		CCVector3 P;
		P.x = -1 * center_point_x;
		P.y = -1 * center_point_y;
		P.z = -1 * center_point_z;
		cloud->translate(P);
	}
	else
	{
		CCVector3 P;
		P.x = 0.0;
		P.y = 0.0;
		P.z = num;
		cloud->translate(P);
	}
}

//添加 Chenmobenmo 241106
void sm2ccConverter::Yalign(ccPointCloud* cloud)
{
	CCVector3 P;
	P.x = 0;
	P.y = -1 * (cloud->min_y + cloud->max_y) / 2;
	P.z = 0;
	cloud->translate(P);
	cloud->min_y += P.y;
	cloud->max_y += P.y;
}

//添加 Chenmobenmo 241107
void sm2ccConverter::Xalign(ccPointCloud* cloud)
{
	CCVector3 P;
	P.x = -1 * (cloud->min_x + cloud->max_x) / 2;
	P.y = 0;
	P.z = 0;
	cloud->translate(P);
	cloud->min_x += P.x;
	cloud->max_x += P.x;
}

//添加 Chenmobenmo 240929
void sm2ccConverter::revolve(ccPointCloud* cloud, int rad, std::string name)
{
	cloud->revolve(rad, name);
	//需要修改1 Chenmobenmo 241002
	center_point_x = cloud->barycenter_point_x; //注释 Chenmobenmo 241002
	center_point_y = cloud->barycenter_point_y; //注释 Chenmobenmo 241002
	center_point_z = cloud->barycenter_point_z; //注释 Chenmobenmo 241002

	//添加 Chenmobenmo 241002 //需要修改2
	//if (name == "LEFT")
	//{
	//	center_point_x = ccVitalZ::L_VITAL_X;
	//	center_point_y = ccVitalZ::L_VITAL_Y;
	//	center_point_z = ccVitalZ::L_VITAL_Z;
	//}
	//if (name == "INFRONT")
	//{
	//	center_point_x = ccVitalZ::I_VITAL_X;
	//	center_point_y = ccVitalZ::I_VITAL_Y;
	//	center_point_z = ccVitalZ::I_VITAL_Z;
	//}
	//if (name == "RIGHT")
	//{
	//	center_point_x = ccVitalZ::R_VITAL_X;
	//	center_point_y = ccVitalZ::R_VITAL_Y;
	//	center_point_z = ccVitalZ::R_VITAL_Z;
	//}
}

//添加 Chenmobenmo 241002
void sm2ccConverter::Get_barycenter_point(ccPointCloud* cloud, int rad, std::string name)
{
	cloud->Get_barycenter_point(rad, name);
}

//添加 Chenmobenmo 241003
void sm2ccConverter::Get_MM_XZ(ccPointCloud* cloud)
{
	cloud->Get_MM_XZ();
}

bool sm2ccConverter::addNormals(ccPointCloud *cloud)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	pcl::PointCloud<OnlyNormals>::Ptr pcl_cloud_normals (new pcl::PointCloud<OnlyNormals>);
	FROM_PCL_CLOUD(*m_sm_cloud, *pcl_cloud_normals);

	if (!cloud->reserveTheNormsTable())
		return false;

	size_t pointCount = GetNumberOfPoints(m_sm_cloud);

	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3 N(	static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_x),
						static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_y),
						static_cast<PointCoordinateType>(pcl_cloud_normals->at(i).normal_z) );

		cloud->addNorm(N);
	}

	cloud->showNormals(true);
	
	return true;
}

bool sm2ccConverter::addRGB(ccPointCloud * cloud)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	pcl::PointCloud<OnlyRGB>::Ptr pcl_cloud_rgb (new pcl::PointCloud<OnlyRGB>);
	FROM_PCL_CLOUD(*m_sm_cloud, *pcl_cloud_rgb);
	size_t pointCount = GetNumberOfPoints(m_sm_cloud);
	if (pointCount == 0)
		return true;
	if (!cloud->reserveTheRGBTable())
		return false;


	//loop
	for (size_t i = 0; i < pointCount; ++i)
	{
		ccColor::Rgb C(	static_cast<ColorCompType>(pcl_cloud_rgb->points[i].r),
						static_cast<ColorCompType>(pcl_cloud_rgb->points[i].g),
						static_cast<ColorCompType>(pcl_cloud_rgb->points[i].b) );
		cloud->addColor(C);
	}

	cloud->showColors(true);

	return true;
}

bool sm2ccConverter::addScalarField(ccPointCloud* cloud, const std::string& name, bool overwrite_if_exist/*=true*/)
{
	assert(m_sm_cloud && cloud);
	if (!m_sm_cloud || !cloud)
		return false;

	//if the input field already exists...
	int id = cloud->getScalarFieldIndexByName(name.c_str());
	if (id >= 0)
	{
		if (overwrite_if_exist)
			//we simply delete it
			cloud->deleteScalarField(id);
		else
			//we keep it as is
			return false;
	}

	size_t pointCount = GetNumberOfPoints(m_sm_cloud);

	//create new scalar field
	ccScalarField* cc_scalar_field = new ccScalarField(name.c_str());
	if (!cc_scalar_field->reserveSafe(static_cast<unsigned>(pointCount)))
	{
		cc_scalar_field->release();
		return false;
	}

	//get PCL field
	int field_index = pcl::getFieldIndex(*m_sm_cloud, name);
	PCLScalarField& pclField = m_sm_cloud->fields[field_index];
	//temporary change the name of the given field to something else -> S5c4laR should be a pretty uncommon name,
	pclField.name = std::string("S5c4laR");

	switch (pclField.datatype)
	{
	case PCLScalarField::FLOAT32:
	{
		pcl::PointCloud<FloatScalar>::Ptr pcl_scalar(new pcl::PointCloud<FloatScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pcl_scalar->points[i].S5c4laR);
			cc_scalar_field->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::FLOAT64:
	{
		pcl::PointCloud<DoubleScalar>::Ptr pcl_scalar(new pcl::PointCloud<DoubleScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pcl_scalar->points[i].S5c4laR);
			cc_scalar_field->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::INT16:
	{
		pcl::PointCloud<ShortScalar>::Ptr pcl_scalar(new pcl::PointCloud<ShortScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pcl_scalar->points[i].S5c4laR);
			cc_scalar_field->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::UINT16:
	{
		pcl::PointCloud<UShortScalar>::Ptr pcl_scalar(new pcl::PointCloud<UShortScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pcl_scalar->points[i].S5c4laR);
			cc_scalar_field->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::UINT32:
	{
		pcl::PointCloud<UIntScalar>::Ptr pcl_scalar(new pcl::PointCloud<UIntScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pcl_scalar->points[i].S5c4laR);
			cc_scalar_field->addElement(scalar);
		}
	}
	break;

	case PCLScalarField::INT32:
	{
		pcl::PointCloud<IntScalar>::Ptr pcl_scalar(new pcl::PointCloud<IntScalar>);
		FROM_PCL_CLOUD(*m_sm_cloud, *pcl_scalar);

		for (size_t i = 0; i < pointCount; ++i)
		{
			ScalarType scalar = static_cast<ScalarType>(pcl_scalar->points[i].S5c4laR);
			cc_scalar_field->addElement(scalar);
		}
	}
	break;

	default:
		ccLog::Warning(QString("[PCL] Field with an unmanaged type (= %1)").arg(pclField.datatype));
		cc_scalar_field->release();
		return false;
	}

	cc_scalar_field->computeMinAndMax();
	cloud->addScalarField(cc_scalar_field);
	cloud->setCurrentDisplayedScalarField(0);
	cloud->showSF(true);

	//restore old name for the scalar field
	m_sm_cloud->fields[field_index].name = name;

	return true;
}