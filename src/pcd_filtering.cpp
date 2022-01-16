#include "sim_kinect/model_utils.h"
#include "sim_kinect/objectKinect.h"
#include "sim_kinect/interaction_utils.h"
#include "sim_kinect/pcl_utils.h"

#include <pcl/pcl_config.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <boost/filesystem.hpp>

#include <fstream>


#ifdef HAVE_OMP
#include <omp.h>
#endif

using namespace std;


typedef pcl::PointXYZI PointT;

int load_single_pcd(string filedir, pcl::PointCloud<PointT>::Ptr cloud)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<PointT> (filedir, *cloud) == -1) //* load the file
  		{
		    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		    return (-1);
		  }
	  std::cout << "Loaded "
	            << cloud->width * cloud->height
	            << " data points from " << filedir
	            << std::endl;
    return 0;
}


pcl::PointCloud<PointT> loadBilateralFilter(string filedir, double sigma_s=1.0, double sigma_r=0.05)
{
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

  // Load cloud
	load_single_pcd(filedir, cloud);

	// Output Cloud = Input Cloud
	pcl::PointCloud<PointT> outcloud;

	// Set up KDTree
	pcl::search::KdTree< PointT >::Ptr tree (new pcl::search::KdTree< PointT >);

	// Set up filter parameters
	// double sigma_s = 0.5;
	// double sigma_r = 10;
	// Init filter
	pcl::BilateralFilter<PointT> bf;
	bf.setInputCloud (cloud);
	bf.setSearchMethod (tree);
	bf.setHalfSize (sigma_s);
	bf.setStdDev (sigma_r);
	bf.filter (outcloud);
	return outcloud;

	// pcl::io::savePCDFile (savedir.c_str (), outcloud);
}



int main(int argc, char **argv)
{
	string PROJECT_DIR = "/home/senjing/3d-vision/";
	string BASE_DIR = PROJECT_DIR + "stl_sampling/";
	string DATA_DIR = PROJECT_DIR + "data/";
	string FILELSIT = "filelist";
	string SAVE_DIR = "/tmp/filtered_pcd/";

  srand(time(NULL));
  cout << PCL_VERSION_PRETTY << endl;
  cgalVersion();

  string filedir;
  string savedir;

  // get filelist name
	if ((argv[1])!=NULL)
	{
		DATA_DIR = DATA_DIR + argv[1] + "/";
		SAVE_DIR = SAVE_DIR + argv[1] + "/";
		boost::filesystem::create_directories(SAVE_DIR);
		if ((argv[2])!=NULL)
		{
			FILELSIT = argv[2];
		}
	}
	else
	{
		DATA_DIR = DATA_DIR + "kinect/";
		SAVE_DIR = SAVE_DIR + "kinect/";
		boost::filesystem::create_directories(SAVE_DIR);
		cout << "Did not type in datadir, run with demo mode" << endl;
	}

	if (!boost::filesystem::exists( DATA_DIR + FILELSIT ))
	{
		cout << "Can't find " << FILELSIT << endl;
		return 0;
	}

	// Read filelist
	ifstream filelist_file(DATA_DIR + FILELSIT);

	if (filelist_file.is_open())
	{
		string file_name;
		int count=0;
		while (getline(filelist_file, file_name))
		{
			pcl::PointCloud<PointT> outcloud = loadBilateralFilter( DATA_DIR + file_name );
			string savedir = SAVE_DIR + file_name;
			pcl::io::savePCDFile (savedir.c_str(), outcloud);
			count ++;
		}
	}
	return 0;
}