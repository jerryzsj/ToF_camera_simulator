/*********************************************************************
*		This is a object class for Kinect
*		
*		Author: Senjing Zheng
*		Email: senjing.zheng@gmail.com
*		Create: 17-01-2019
*		Update: 16-06-2019
*   
*********************************************************************/


#ifndef OBJECTKINECT_H_
#define OBJECTKINECT_H_

#include "sim_kinect/std_utils.h"
#include "sim_kinect/model_utils.h"
#include "sim_kinect/eigen_utils.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector4f Vector4f;

class Kinect
{
	public:
		Kinect();
		Kinect(float rot_z, float rot_x);
		Kinect(string tag_);
		Kinect(float distance_);
		Kinect(string tag_, float distance_);
		Kinect(const Kinect &kinect_);
		~Kinect();

		// Init camera
		void InitPose();
		void InitPosition();
		void InitRay();
		void InitKinect();
		
		// Set parameters
		void SetTag(string tag_);
		void SetDistance(float distance_);
		void SetRandTransform();
		void SetDegreeTransform(float rot_z, float rot_x);


		// Update Kinect
		void UpdatePose();
		void UpdatePosition();
		void UpdateRays();
		void UpdateKinect();

		Vector3f camera_position;
		Vector3f camera_pose;
		std::vector<Vector3f> camera_ray;

	// private:
		Vector3f init_camera_position = Vector3f(0,0,0);
		std::vector<Vector3f> init_camera_ray;  // init camera rays from origin
		Vector3f init_camera_pose ;  // init camera pose from origin
		Eigen::Matrix3f camera_transform = Eigen::Matrix3f::Identity();

		std::vector<Eigen::Matrix3f> arrTransform;

		string tag = "720p";
		float camera_distance = 500.0;
		int camera_width = 1280; //640
		int camera_height = 720; //480
		int camera_f = 1160; //580
};

Kinect::Kinect()
{
	this->InitKinect();
	this->UpdateKinect();
}

Kinect::Kinect(float rot_z, float rot_x)
{
	this->InitKinect();
	this->SetDegreeTransform(rot_z, rot_x);
}

Kinect::Kinect(string tag_)
{
	this->SetTag(tag_);
}

Kinect::Kinect(float distance_)
{
	this->InitKinect();
	this->SetDistance(distance_);
}

Kinect::Kinect(string tag_, float distance_)
{
	this->SetTag(tag_);
	this->SetDistance(distance_);
}

Kinect::Kinect(const Kinect &kinect_)
{
	this->init_camera_position = kinect_.init_camera_position;
	this->init_camera_ray= kinect_.init_camera_ray;  // init camera rays from origin
	this->init_camera_pose= kinect_.init_camera_pose;  // init camera pose from origin
	this->camera_transform= kinect_.camera_transform;
	this->tag = kinect_.tag;
	this->camera_distance= kinect_.camera_distance;
	this->camera_width = kinect_.camera_width; //640
	this->camera_height = kinect_.camera_height; //480
	this->camera_f = kinect_.camera_f; //580
}

Kinect::~Kinect()
{
}

//--------------------------------------------------------------------------//
// Initiate init_camera_ray & init_camera_pose
void Kinect::InitPose()
{
	this->init_camera_pose=(Vector3f(0.0, float(this->camera_f), 0.0)).normalized();
	return;
}

void Kinect::InitPosition()
{
	this->camera_position = this->init_camera_position - (this->camera_distance * this->camera_pose);
	return;
}

void Kinect::InitRay()
{
	std::vector<Vector3f>().swap(this->init_camera_ray);  // empty def_camera_ray
	for (int u = 0; u < this->camera_height; u++)
	{
		for (int v = 0; v < this->camera_width; v++)
		{
			float x = float(v) + 0.5 - float(this->camera_width)/2.0; 
			float y = float(this->camera_f);
			float z = float(this->camera_height)/2.0 - 0.5 - float(u);
			this->init_camera_ray.push_back(Vector3f(x, y, z).normalized()); 
		}
	}
	this->camera_ray.resize(this->init_camera_ray.size());
	return;
}

void Kinect::InitKinect()
{
	this->InitPose();
	this->InitPosition();
	this->InitRay();
	return;
}



//--------------------------------------------------------------------------//
// Modifying Kinect parameters
void Kinect::SetTag(string tag_)
{
	this->tag = tag_;
	if (this->tag == "720p"){this->camera_width = 1280; this->camera_height = 720; this->camera_f = 1160;}
	if (this->tag == "480p"){this->camera_width = 640; this->camera_height = 480; this->camera_f = 580;}
	this->InitKinect();
	this->UpdateKinect();
	return;
}

void Kinect::SetDistance(float distance_)
{
	this->camera_distance = distance_;
	this->UpdatePosition();
	this->UpdateKinect();
	return;
}

void Kinect::SetRandTransform() // randomly set camera trasnform
{
	float roll = float(rand() % 200) / 100.0;
	float pitch = float(rand() % 200) / 100.0;
	float yaw = float(rand() % 200) / 100.0;
	this->camera_transform = rotateRoll(roll*M_PI) * rotatePitch(pitch*M_PI) * rotateYaw(yaw*M_PI);
	this->UpdateKinect();
	return;
}

void Kinect::SetDegreeTransform(float rot_z, float rot_x) 
{
	float roll = rot_z/180.0;
	float pitch = rot_x/180.0;
	this->camera_transform = rotateRoll(roll*M_PI) * rotatePitch(pitch*M_PI);
	this->UpdateKinect();
	return;
}

//--------------------------------------------------------------------------//
// Update Kinect parameters
void Kinect::UpdatePose()
{
	this->camera_pose = (this->camera_transform * this->init_camera_pose).normalized();
	return;
}

void Kinect::UpdatePosition()
{
	this->camera_position = this->init_camera_position - (this->camera_distance * this->camera_pose);
	return;
}

void Kinect::UpdateRays()
{
	std::vector<Vector3f>().swap(this->camera_ray);
	for (int i=0; i< this->init_camera_ray.size(); i++)
	{
		this->camera_ray.push_back((this->camera_transform * this->init_camera_ray[i]).normalized());
	}
	return;
}

void Kinect::UpdateKinect()
{
	this->UpdatePose();
	this->UpdatePosition();
	this->UpdateRays();
}
//--------------------------------------------------------------------------//




// //--------------------------------------------------------------------------//
// // Batch generation for array transformations
// void Kinect::RandArrTransform() // 
// {
// 	Eigen::Matrix3f transform_ = Eigen::Matrix3f::Identity();
// 	Eigen::Matrix3f temp_ = Eigen::Matrix3f::Identity();
// 	float roll = float(rand() % 200) / 100.0;
// 	float pitch = float(rand() % 200) / 100.0;
// 	float yaw = float(rand() % 200) / 100.0;
// 	transform_ = rotateRoll(roll*M_PI) * rotatePitch(pitch*M_PI) * rotateYaw(yaw*M_PI);
// 	for (int i=0; i< this->arrTransform.size(); i++)
// 	{
// 		temp_ = this->arrTransform[i];
// 		this->arrTransform[i] = transform_ * temp_;
// 	}
// 	return;
// }


// void Kinect::InitArrTransform() // 
// {
// 	std::vector<Eigen::Matrix3f>().swap(this->arrTransform);
// 	Eigen::Matrix3f transform_ = Eigen::Matrix3f::Identity();
// 	for (int z_a=0; z_a < 8; z_a++)
// 	{
// 		transform_ = rotateRoll(0.25*z_a*M_PI) * rotatePitch(0);
// 		this->arrTransform.push_back(transform_);
// 	}

// 	for (int z_a = 0; z_a < 4; z_a++)
// 	{
// 		transform_ = rotateRoll(0.5*z_a*M_PI) * rotatePitch(-0.25 * M_PI);
// 		this->arrTransform.push_back(transform_);
// 	}
// 	return;
// }




#endif  // OBJECTKINECT_H_ 