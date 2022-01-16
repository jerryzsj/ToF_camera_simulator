#include "sim_kinect/model_utils.h"
#include "sim_kinect/objectKinect.h"
#include "sim_kinect/interaction_utils.h"
#include "sim_kinect/pcl_utils.h"

#include <pcl/pcl_config.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/filesystem.hpp>


#ifdef HAVE_OMP
#include <omp.h>
#endif

using namespace std;



int main(int argc, char const *argv[])
{
	srand(time(NULL));

	Kinect myKinect;

	Kinect kinet2 = myKinect;

	return 0;
}