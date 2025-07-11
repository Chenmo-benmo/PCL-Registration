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
//#                        COPYRIGHT: Luca Penasa                          #
//#                                                                        #
//##########################################################################

#ifndef Q_PCL_PLUGIN_SM2CC_H
#define Q_PCL_PLUGIN_SM2CC_H

//Local
#include "PCLCloud.h"

//system
#include <list>

class ccPointCloud;

//! PCL to CC cloud converter
/** NOTE: THIS METHOD HAS SOME PROBLEMS. IT CANNOT CORRECTLY LOAD NON-FLOAT FIELDS!
	THIS IS DUE TO THE FACT THE POINT TYPE WITH A SCALAR WE USE HERE IS FLOAT
	IF YOU TRY TO LOAD A FIELD THAT IS INT YOU GET A PCL WARN!
**/
class sm2ccConverter
{
public:

	//! Default constructor
	sm2ccConverter(PCLCloud::Ptr sm_cloud);

	//! Converts input cloud (see constructor) to a ccPointCloud
	ccPointCloud* getCloud(bool cc = false/*添加 Chenmobenmo 240929*/,
						   int rad = 0/*添加 Chenmobenmo 240930*/,
						   std::string name = ""/*添加 Chenmobenmo 241001*/);

	bool addXYZ        (ccPointCloud *cloud);
	bool addNormals    (ccPointCloud *cloud);
	bool addRGB        (ccPointCloud *cloud);
	bool addScalarField(ccPointCloud *cloud, const std::string& name, bool overwrite_if_exist = true);

	void filter(ccPointCloud* cloud, float probability/*删除周围点数百分比*/, float gridLength/*网格大小*/); //添加 Chenmobenmo 241020
	void translate(ccPointCloud* cloud, float num = 0);	//添加 Chenmobenmo 240929
	void Yalign(ccPointCloud* cloud); //添加 Chenmobenmo 241106
	void Xalign(ccPointCloud* cloud); //添加 Chenmobenmo 241107
	void revolve(ccPointCloud* cloud, int rad, std::string name = "");		//添加 Chenmobenmo 240929
	void Get_barycenter_point(ccPointCloud* cloud, int rad, std::string name = ""); //添加 Chenmobenmo 241002
	void Get_MM_XZ(ccPointCloud* cloud); //添加 Chenmobenmo 241003

private:

	//! Associated PCL cloud
	PCLCloud::Ptr m_sm_cloud;

public: //添加 Chenmobenmo 240930
	float center_point_x;
	float center_point_y;
	float center_point_z;
};

#endif // Q_PCL_PLUGIN_SM2CC_H
