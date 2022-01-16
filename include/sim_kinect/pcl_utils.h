/*********************************************************************
*		This is an util header file for interaction between ray and mesh
*		
*		Reference < Robot Arm Pose Estimation through Pixel-Wise Part Classification. 
*		Jeannette Bohg, Javier Romero, Alexander Herzog and Stefan Schaal. 
*		Proceedings of the 2014 IEEE International Conference on Robotics and Automation. pp 3143--3150. 2014. >
*		
*		Author: Senjing Zheng
*		Email: senjing.zheng@gmail.com
*		Create: 21-01-2019
*		Update: 21-01-2019
*
*********************************************************************/


#ifndef PCL_UTILS_H_
#define PCL_UTILS_H_

#include <iostream>
#include <pcl/pcl_config.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "sim_kinect/std_utils.h"


typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGBType;

void demo()
{
	CloudType::Ptr cloud (new CloudType);
	CloudType::PointType p;
	p.x = 1; p.y = 2; p.z = 3;
	cloud->push_back(p);
	return;
}

void Show_PCL(CloudType::Ptr cloud, string viewer_name)
{
	pcl::visualization::CloudViewer viewer(viewer_name);
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
  {
  }
}

void Show_PCL_Color(CloudRGBType::Ptr cloud, string viewer_name)
{
	pcl::visualization::CloudViewer viewer(viewer_name);
	viewer.showCloud(cloud);
	while(!viewer.wasStopped())
	{

	}
}

CloudType::Ptr Read_PCL(string filename)
{
	CloudType::Ptr cloud (new CloudType);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file");
	}
	std::cout << "Loaded " << filename << endl;
	return cloud;
}

void Show_PCD(string filename)
{
	CloudType::Ptr cloud = Read_PCL(filename);
	Show_PCL(cloud, filename);
}

#endif