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

string PROJECT_DIR = "/home/senjing/3d-vision/";
string BASE_DIR = PROJECT_DIR + "ToF_camera_simulator/";
string DATA_DIR = BASE_DIR + "data/mech12_stl/";
string SAVE_DIR = BASE_DIR + "data/three_cam_pcd/";
bool COMBINE = false;

string FILELSIT_DIR = "filelist";
float preset_cam_diatance = 500.0;


void load_single_stl(string filename, MeshAndModel* model_, TreeAndTri* search_)
{
    string filePath = DATA_DIR + filename;
    // load meshes from stl files
    loadMeshSingle(model_->myMeshes, filePath);
    // // init vertices for model
    initMeshVertices(model_->myVertices, model_->myMeshes);
    // init number of vertices
    initNumVertices(model_);
    // init number of faces
    initNumFaces(model_);
    // init transformation matrix
    initTransform(model_);
    //cout << myModel->myTransform.matrix() << endl;

    // init vertices for search tree
    initTreeVertices(search_, model_);
    // init faces for search tree
    initTreeFaces(search_, model_);
    // init part id for search tree
    initPartIDs(search_, model_);
    // init tree
    initTree(search_);
    setTreeCoorOrigin(search_);
    return;
}

void scan_single_stl(string filename, int num_samples)
{    
    MeshAndModel* myModel = new MeshAndModel;
    TreeAndTri* mySearch = new TreeAndTri;
    Kinect* myKinect = new Kinect(preset_cam_diatance);

    load_single_stl(filename, myModel, mySearch);

    string stl_name;
    stl_name = filename;
    stl_name.resize(filename.size()-4);

    cout << stl_name << endl;
    
    cout << "Converting " << stl_name << endl;

    string save_dir = SAVE_DIR + stl_name + "/";
    boost::filesystem::create_directories(save_dir);
    
    string TransM_DIR = save_dir + "transMatrix.dat";
    std::ofstream file(TransM_DIR);
    
    string origin_name = "origin";

    Three_Cam(mySearch, myKinect, save_dir, origin_name);
    std::ofstream filelist_file(save_dir + "filelist");
    for(int i =0; i<num_samples; ++i)
    {
        
        if (filelist_file.is_open())
        {
            filelist_file << std::to_string(i) << ".pcd\n";
        }
        
        TreeAndTri* search_ = new TreeAndTri;
        search_->points.assign(mySearch->points.begin(), mySearch->points.end());
        search_->indices.assign(mySearch->indices.begin(), mySearch->indices.end());
        search_->part_ids.assign(mySearch->part_ids.begin(), mySearch->part_ids.end());
        search_->triangles.assign(mySearch->triangles.begin(), mySearch->triangles.end());
        search_->transform = mySearch->transform;
        randTreeTransform(search_);

        Three_Cam(search_, myKinect, save_dir, std::to_string(i));

        if (file.is_open())
        {
            file << search_->eigen_transform.format(CommaInitFmt);
        }
    }
    file.close();
    filelist_file.close();
    cout << "done with " << stl_name << endl;
    return;
}



int main(int argc, char **argv)
{
    cout << "Usage: ./three_cam_D500 NUM_SAMPLE FILELIST" << endl;
    cout << "DATA_DIR" << DATA_DIR <<endl;
    srand(time(NULL));
    cout << "PCL version:" << PCL_VERSION_PRETTY << endl;
    cgalVersion();

    boost::filesystem::create_directories(SAVE_DIR);
    
    int NUM_SAMPLE = stoi(argv[1]);

    // get filelist name
    if ((argv[2])!=NULL)
    {
        FILELSIT_DIR = argv[2];
        std::ifstream file_filelist(DATA_DIR + FILELSIT_DIR);
        std::vector<string> FILELIST;
        int FILELIST_LEN = 0;
        string temp_line;
        while(getline(file_filelist, temp_line))
        {
            FILELIST.push_back(temp_line);
            ++FILELIST_LEN;
        }
        printf("Got %d files\n", FILELIST_LEN);
#if HAVE_OMP
#pragma omp parallel for collapse(1)
#endif
        for(int i = 0; i < FILELIST_LEN; ++i)
        {
            scan_single_stl(FILELIST[i].c_str(), NUM_SAMPLE);
        }
    }
    else
    {
        std::ifstream file_filelist(DATA_DIR + FILELSIT_DIR);
        std::vector<string> FILELIST;
        int FILELIST_LEN = 0;
        string temp_line;
        while(getline(file_filelist, temp_line))
        {
            FILELIST.push_back(temp_line);
            ++FILELIST_LEN;
        }
        printf("Got %d files\n", FILELIST_LEN);
#if HAVE_OMP
#pragma omp parallel for collapse(1)
#endif
        for(int i = 0; i < FILELIST_LEN; ++i)
        {
            scan_single_stl(FILELIST[i].c_str(), NUM_SAMPLE);
        }
        
    }

    cout << "done with all" << endl;

    return 0;
}

