/*********************************************************************
*		This is an util header file for eigen
*		
*		Author: Senjing Zheng
*		Email: senjing.zheng@gmail.com
*		Create: 18-01-2019
*		Update: 18-01-2019
*
*********************************************************************/


#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <math.h>

#include <pcl/pcl_config.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "sim_kinect/pcl_utils.h"


Eigen::Matrix3f rotateRoll(float roll)
{
	Eigen::Matrix3f transform_;
	transform_ <<  cos(roll), sin(roll), 0,
								-sin(roll), cos(roll), 0,
								0, 0, 1;
	// cout << "roll";
	return transform_;
}

Eigen::Matrix3f rotatePitch(float pitch)
{
	Eigen::Matrix3f transform_;
	transform_ <<  1, 0, 0, 
								0, cos(pitch), sin(pitch),
								0, -sin(pitch), cos(pitch);
	// cout << "pitch";
	return transform_;
}


Eigen::Matrix3f rotateYaw(float yaw)
{
	Eigen::Matrix3f transform_;
	transform_ <<  cos(yaw), 0, -sin(yaw),
								0, 1, 0,
								sin(yaw), 0, cos(yaw);
	// cout << "yaw";
	return transform_;
}


Eigen::Matrix4f affRoll(float roll)
{
	Eigen::Matrix4f transform_;
	transform_ <<  cos(roll), sin(roll), 0,0,
					-sin(roll), cos(roll), 0,0,
					0, 0, 1,0,
					0,0,0,1;
	// cout << "roll";
	return transform_;
}

Eigen::Matrix4f affPitch(float pitch)
{
	Eigen::Matrix4f transform_;
	transform_ <<  1, 0, 0, 0,
					0, cos(pitch), sin(pitch),0,
					0, -sin(pitch), cos(pitch),0,
					0,0,0,1;
	// cout << "pitch";
	return transform_;
}


Eigen::Matrix4f affYaw(float yaw)
{
	Eigen::Matrix4f transform_;
	transform_ <<  cos(yaw), 0, -sin(yaw),0,
					0, 1, 0,0,
					sin(yaw), 0, cos(yaw), 0,
					0,0,0,1;
	// cout << "yaw";
	return transform_;
}


Eigen::Vector3f eigenRand3D(int max_d)
{
	Eigen::Vector3f vector_ = Eigen::Vector3f::Identity();
	float x_ = float(rand() % (max_d));
	float y_ = float(rand() % (max_d));
	float z_ = float(rand() % (max_d));
	vector_ << x_, y_, z_;
	return vector_;
}

Eigen::Vector3f eigenRand3DNorm()
{
	Eigen::Vector3f vector_ = Eigen::Vector3f::Identity();
	float x_ = float(rand()%200) - 100.0;
	float y_ = float(rand()%200) - 100.0;
	float z_ = float(rand()%200) - 100.0;
	vector_ << x_, y_, z_;
	return vector_.normalized();
}

Eigen::Vector3f eigenRandCamPose()
{
	Eigen::Vector3f vector_ = Eigen::Vector3f::Identity();
	float x_ = float(rand()%200) - 100.0;
	float y_ = float(rand()%200) - 100.0;
	float z_ = float(rand()%100) - 200.0;
	vector_ << x_, y_, z_;
	return vector_.normalized();
}



void eigen_to_pcl(std::vector<Eigen::RowVector4f> &eigen_cloud, CloudType::Ptr cloud)
{
	for (int i = 0; i < eigen_cloud.size(); i++)
	{
		cloud->push_back(pcl::PointXYZ(eigen_cloud[i](0), eigen_cloud[i](1), eigen_cloud[i](2)));
	}
	return;
}

Eigen::Matrix3f eigen_rand_rotate()
{
	Eigen::Matrix3f transform_ = Eigen::Matrix3f::Identity();
	float roll = float(rand() % 200) / 100.0;
	float pitch = float(rand() % 200) / 100.0;
	float yaw = float(rand() % 200) / 100.0;
	transform_ = rotateRoll(roll*M_PI) * rotatePitch(pitch*M_PI) * rotateYaw(yaw*M_PI);
	return transform_;
}

Eigen::Matrix4f eigen_rand_affrotate()
{
	Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
	float roll = float(rand() % 200) / 100.0;
	float pitch = float(rand() % 200) / 100.0;
	float yaw = float(rand() % 200) / 100.0;
	transform_ = affRoll(roll*M_PI) * affPitch(pitch*M_PI) * affYaw(yaw*M_PI);
	return transform_;
}

Eigen::Matrix4f eigen_rand_afftranslate(int max_x, int max_y, int max_z)
{
	Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
	float x;
	if (max_x==0)
	{
		x = 0.0;
	}
	else
	{
		x = (float(rand() % (20*max_x)) - 10.0*max_x)/10.0;
	}
	float y;
	if (max_y==0)
		y = 0.0;
	else
		y = (float(rand() % (20*max_y)) - 10.0*max_y)/10.0;
	float z;
	if (max_z==0)
	{
		z = 0.0;
	}
	else
	{
		z = (float(rand() % (10*max_z)))/10.0;
	}
	transform_ <<   1, 0, 0,x,
					0, 1, 0,y,
					0, 0, 1,z,
					0,0,0,1;
	return transform_;
}

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, "	", "	", "", "", "", "\n");

#endif