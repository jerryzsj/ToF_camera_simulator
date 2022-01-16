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


#ifndef INTERACTION_UTILS_H_
#define INTERACTION_UTILS_H_

#include "sim_kinect/eigen_utils.h"
#include "sim_kinect/objectKinect.h"
#include "sim_kinect/model_utils.h"
#include "sim_kinect/std_utils.h"
#include "sim_kinect/pcl_utils.h"

#include <boost/filesystem.hpp>

// const string DATA_PREP_PATH = "../data_prep/";
// const string DATA_PATH = "../data/";

void Kinect_Mesh_Interaction(Kinect* kinect_, TreeAndTri *search_, std::vector<Eigen::RowVector4f> &eigen_cloud)
{
    // std::cout << "have " << search_->triangles.size() << " tree faces" << std::endl;
	for (int j=0; j<kinect_->camera_ray.size(); j++)
    {
        // cout << kinect_->camera_ray[j].x()<< "  " << kinect_->camera_ray[j].y()<< "  " <<kinect_->camera_ray[j].z() << endl;
        // check if there is any intersection of the ray with an object by do_intersect
        uint32_t reach_mesh = search_->tree.do_intersect(Ray ( Point(kinect_->camera_position.x(), kinect_->camera_position.y(), kinect_->camera_position.z()), Vector(kinect_->camera_ray[j].x(), kinect_->camera_ray[j].y(), kinect_->camera_ray[j].z()) ) );
        if (reach_mesh)
        {
            // if there is one or many intersections, order them according to distance to camera and continue computation with closest
            std::list<Object_and_Primitive_id> intersections;
            search_->tree.all_intersections((Ray(Point(kinect_->camera_position.x(), kinect_->camera_position.y(), kinect_->camera_position.z()), Vector(kinect_->camera_ray[j].x(), kinect_->camera_ray[j].y(), kinect_->camera_ray[j].z()))), std::back_inserter(intersections));
            if(!intersections.empty())
            {
                //cout << "intersections not empty!" << i <<endl;
                std::list<Object_and_Primitive_id>::const_iterator it;
                Point min_p(0,0,0);
                float min_dist = std::numeric_limits<float>::infinity();
                float min_id = -1;
                for (it = intersections.begin(); it != intersections.end(); ++it) 
                {
                  CGAL::Object object = it->first;
                  std::size_t triangle_id = std::distance(search_->triangles.begin(),it->second);
                  assert( search_->triangles[triangle_id] == *(it->second) ); 
                  Point point;
                    if(CGAL::assign(point,object))
                    {
                        float dist = CGAL::squared_distance(point, Point(kinect_->camera_position.x(), kinect_->camera_position.y(), kinect_->camera_position.z()));
                        if(dist<min_dist)
                        {
                            // distance to point
                            min_dist = dist;
                            // intersection coordinates
                            min_p = point;
                            // label of the intersected object (will be zero for this simple case)
                            min_id = search_->part_ids[triangle_id];
                        }
                        // else 
                        // {
                        //     // std::cout << "Intersection object is NOT a point ?????" << std::endl;
                        // }
                    }
                }
                if(min_id != -1)
                {
                    // cloud.points[i*640*480 +j].x = min_p.x();
                    // cloud.points[i*640*480 +j].y = min_p.y();
                    // cloud.points[i*640*480 +j].z = min_p.z();
                    eigen_cloud.push_back(Eigen::RowVector4f(min_p.x(), min_p.y(), min_p.z(), min_id));
                    // cloud.push_back(min_p.x(), min_p.y(), min_p.z());
                }
            }
        }
    }
	// cout << "finish interaction" << endl;
 //    cout << INTERVAL;
	return;
}


// generate point cloud with fixed 12 cams, 4 cams and 1 cam
void one_to_one_fix_cam(TreeAndTri* mySearch, Kinect* myKinect, string save_dir, string pcd_name)
{
    CloudType::Ptr cloud(new CloudType);
    CloudType::Ptr cloud_4(new CloudType);
    CloudType::Ptr cloud_1(new CloudType);
    std::vector<Eigen::RowVector4f> eigen_cloud;
    std::vector<Eigen::RowVector4f> eigen_cloud_4;
    std::vector<Eigen::RowVector4f> eigen_cloud_1;

    for (int i =0; i< 12; i++)
    {
        myKinect->SetDegreeTransform((i/12.0*360.0), 0.0);
        Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
        if (i > 7)
        {
            Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud_4);
            // cout << "a3" << endl;
        }
        if (i == 0)
        {
            Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud_1);
            // cout << "a4" << endl;
        }
    }
    if(eigen_cloud.size()!=0)
    {
        eigen_to_pcl(eigen_cloud, cloud);
        pcl::io::savePCDFileASCII (save_dir + pcd_name + "_12cam" + ".pcd", *cloud);
    }
    else
    {
        cout << "not reach by 12 cams"<< endl;
    }

    if(eigen_cloud_4.size()!=0)
    {
        eigen_to_pcl(eigen_cloud_4, cloud_4);
        pcl::io::savePCDFileASCII (save_dir + pcd_name + "_4cam" + ".pcd", *cloud_4);
    }
    else
    {
        cout << "not reach by 4 cams"<< endl;
    }

    if(eigen_cloud_1.size()!=0)
    {
        eigen_to_pcl(eigen_cloud_1, cloud_1);
        pcl::io::savePCDFileASCII (save_dir + pcd_name + "_1cam" + ".pcd", *cloud_1);        
    }
    else
    {
        cout << "not reach by 1 cams"<< endl;
    }
    

    cout << "get totally " << eigen_cloud.size() << "points in cloud" << endl;
    std::ofstream file(save_dir + pcd_name +".dat");
    if (file.is_open())
    {
        for (int i =0; i < eigen_cloud.size(); i++)
            file << eigen_cloud[i]<< '\n';
    }

    // std::cerr << "Saved " << cloud->points.size() << " data points ";
    // std::cerr << "Saved " << cloud_4->points.size() << " data points ";
    // std::cerr << "Saved " << cloud_1->points.size() << " data points to "<< stl_name << "_" << std::to_string(j) << std::endl;
    // Show_PCL(cloud, "cloud viewer");
    // return EXIT_SUCCESS;
        
    return;
}

// twelve cam
void Full_Cam(TreeAndTri* mySearch, Kinect* myKinect, string save_dir, string pcd_name)
{
    boost::filesystem::create_directories(save_dir);

    CloudType::Ptr cloud(new CloudType);
    std::vector<Eigen::RowVector4f> eigen_cloud;
    for (int i =0; i< 6; i++)
    {
        myKinect->SetDegreeTransform((i/6.0*360.0), 0.0);
        Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    }
    // for (int i =0; i< 8; i++)
    // {
    //     myKinect->SetDegreeTransform((i/8.0*360.0), 22.5);
    //     Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    //     myKinect->SetDegreeTransform((i/8.0*360.0), -22.5);
    //     Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    // }
    // for (int i =0; i< 5; i++)
    // {
    //     myKinect->SetDegreeTransform((i/5.0*360.0), 45.0);
    //     Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    //     myKinect->SetDegreeTransform((i/5.0*360.0), -45.0);
    //     Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    // }
    for (int i =0; i< 3; i++)
    {
        myKinect->SetDegreeTransform((i/3.0*360.0), 67.5);
        Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
        myKinect->SetDegreeTransform((i/3.0*360.0), -67.5);
        Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    }
    // for (int i =0; i< 1; i++)
    // {
    //     myKinect->SetDegreeTransform((i/1.0*360.0), 90);
    //     Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    //     myKinect->SetDegreeTransform((i/1.0*360.0), -90);
    //     Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    // }

    if(eigen_cloud.size()!=0)
    {
        eigen_to_pcl(eigen_cloud, cloud);
        pcl::io::savePCDFileASCII (save_dir + pcd_name + ".pcd", *cloud);
        cout << "get totally " << eigen_cloud.size() << "points in cloud" << endl;
        // std::ofstream file(save_dir + pcd_name +".dat");
        // if (file.is_open())
        // {
        //     for (int i =0; i < eigen_cloud.size(); i++)
        //     {
        //         file << eigen_cloud[i]<< "\n";
        //     }
        // }
    }
}


void Three_Cam(TreeAndTri* mySearch, Kinect* myKinect, string save_dir, string pcd_name)
{
    boost::filesystem::create_directories(save_dir);

    CloudType::Ptr cloud(new CloudType);
    std::vector<Eigen::RowVector4f> eigen_cloud;
    for (int i =0; i< 3; i++)
    {
        myKinect->SetDegreeTransform((i/3.0*360.0), 60.0);
        Kinect_Mesh_Interaction(myKinect, mySearch, eigen_cloud);
    }

    if(eigen_cloud.size()!=0)
    {
        eigen_to_pcl(eigen_cloud, cloud);
        pcl::io::savePCDFileASCII (save_dir + pcd_name + ".pcd", *cloud);
        cout << "get totally " << eigen_cloud.size() << "points in cloud" << endl;
        // std::ofstream file(save_dir + pcd_name +".dat");
        // if (file.is_open())
        // {
        //     for (int i =0; i < eigen_cloud.size(); i++)
        //     {
        //         file << eigen_cloud[i]<< "\n";
        //     }
        // }
    }
}

#endif