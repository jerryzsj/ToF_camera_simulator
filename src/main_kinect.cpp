/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* main_kinect.cpp 
 * Test program that sets up simulator with specific camera parameters 
 * and object mesh. A number of object poses is sampled from which 
 * a desired measured output (depthmap, label image, point cloud) is 
 * generated and stored.
 */

#include <render_kinect/simulate.h>
#include <render_kinect/camera.h>

#include <iostream>
#include <fstream>

/* Sampling of random 6DoF transformations. */
void getRandomTransform(const double &p_x,
			const double &p_y,
			const double &p_z,
			const double &p_angle,
			Eigen::Affine3d &p_tf)
{
  Eigen::Vector3d axis(((double)(rand()%1000))/1000.0,
		       ((double)(rand()%1000))/1000.0,
		       ((double)(rand()%1000))/1000.0);
  Eigen::Vector3d t(p_x*(double)(rand()%2000 -1000)/1000,
		    p_y*(double)(rand()%2000 -1000)/1000,
		    p_z*(double)(rand()%2000 -1000)/1000);
  p_tf = Eigen::Affine3d::Identity();
  p_tf.translate(t);
  p_tf.rotate(Eigen::AngleAxisd( p_angle*(double)(rand()%2000 - 1000)/1000, axis));
}

// main function that generated a number of sample outputs for a given object mesh. 
int main(int argc, char **argv)
{
  
  if(argc<2){
    std::cerr << "Usage: " << argv[0] << "hex_screw.stl" << std::endl;
    exit(-1);
  }
  
  // Get the path to the object mesh model.
  std::string object_models_dir = "../stl_models/";
  std::stringstream full_path;
  full_path << object_models_dir << argv[1];

  // Get the path to the dot pattern
  std::string dot_path = "../data/kinect-pattern_3x3.png";
  
  // Camera Parameters
  render_kinect::CameraInfo cam_info;
  
  cam_info.width = 640;
  cam_info.height = 480;
  cam_info.cx_ = 320;
  cam_info.cy_ = 240;
  
  cam_info.z_near = 0.5;
  cam_info.z_far = 6.0;
  cam_info.fx_ = 580.0;
  cam_info.fy_ = 580.0;
  // baseline between IR projector and IR camera
  cam_info.tx_ = 0.075;

  // Type of noise
  //  cam_info.noise_ = render_kinect::GAUSSIAN;
  //  cam_info.noise_ = render_kinect::PERLIN;
  cam_info.noise_ = render_kinect::NONE;

  // Test Transform
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());
  transform.translate(Eigen::Vector3d(0.0, 0.0, 0.0));
  transform.rotate(Eigen::Quaterniond(0.0,0.0,0.0,1.0));

  // Kinect Simulator
  render_kinect::Simulate Simulator(cam_info, full_path.str(), dot_path);

  // Number of samples
  int frames = 10;
  // Flags for what output data should be generated
  bool store_depth = 1;
  bool store_label = 1;
  bool store_pcd = 1;

  // Storage of random transform
  Eigen::Affine3d noise;
  for(int i=0; i<frames; ++i) {
    
    // sample noisy transformation around initial one
    getRandomTransform(0.02,0.02,0.02,0.05,noise);
    Eigen::Affine3d current_tf = noise*transform;
    
    // give pose and object name to renderer
    Simulator.simulateMeasurement(current_tf, store_depth, store_label, store_pcd);
    
  }

  return 0;
}