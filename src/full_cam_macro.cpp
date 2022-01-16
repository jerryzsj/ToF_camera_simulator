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
string BASE_DIR = PROJECT_DIR + "stl_sampling/";
string DATA_DIR = PROJECT_DIR + "data/mechnet_stl_12/";
string FILELSIT_DIR = "macrolist";
// string SAVE_DIR = "/home/senjing/3d-vision/data/mechnet_pcd_FC/";
string SAVE_DIR = "/tmp/mechnet_14cam_macro/";
bool COMBINE = false;


void load_combine_stl(std::vector<string> filelist, MeshAndModel* model_, TreeAndTri* search_)
{
    std::vector<string> filePath;
    for (int i = 0; i < filelist.size(); i++){
        filePath.push_back(DATA_DIR + filelist[i]);
    }

    // load meshes from stl files
    loadMeshList(model_->myMeshes, filePath);
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

void scan_combine_stl(std::vector<string> filelist, int num_samples)
{    
    MeshAndModel* myModel = new MeshAndModel;
    TreeAndTri* mySearch = new TreeAndTri;
    Kinect* myKinect = new Kinect;

    load_combine_stl(filelist, myModel, mySearch);

    string stl_name;
    stl_name = filelist[0];
    stl_name.resize(filelist[0].size()-6);
    
    cout << "Converting " << stl_name << endl;

    string save_dir = SAVE_DIR + stl_name + "/";
    boost::filesystem::create_directories(save_dir);
    
    string TransM_DIR = save_dir + "transMatrix.dat";
    std::ofstream file(TransM_DIR);
    
    string origin_name = stl_name + "_origin";

    Full_Cam(mySearch, myKinect, save_dir, origin_name);
    for(int i =0; i<num_samples; ++i)
    {
        std::ofstream filelist_file(save_dir + "filelist");
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

        Full_Cam(search_, myKinect, save_dir, std::to_string(i));

        if (file.is_open())
        {
            file << search_->eigen_transform.format(CommaInitFmt);
        }
    }
    file.close();
    cout << "done with " << stl_name << endl;
    return;
}

void load_single_stl(string filelist, MeshAndModel* model_, TreeAndTri* search_)
{
    string filePath = DATA_DIR + filelist;
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

void scan_single_stl(string filelist, int num_samples)
{    
    MeshAndModel* myModel = new MeshAndModel;
    TreeAndTri* mySearch = new TreeAndTri;
    Kinect* myKinect = new Kinect(200.0);

    load_single_stl(filelist, myModel, mySearch);

    string stl_name;
    stl_name = filelist;
    stl_name.resize(filelist.size()-4);
    
    cout << "Converting " << stl_name << endl;

    string save_dir = SAVE_DIR + stl_name + "/";
    boost::filesystem::create_directories(save_dir);
    
    string TransM_DIR = save_dir + "transMatrix.dat";
    std::ofstream file(TransM_DIR);
    
    string origin_name = "origin";

    Full_Cam(mySearch, myKinect, save_dir, origin_name);
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

        Full_Cam(search_, myKinect, save_dir, std::to_string(i));

        if (file.is_open())
        {
            file << search_->eigen_transform.format(CommaInitFmt);
        }
    }
    file.close();
    cout << "done with " << stl_name << endl;
    return;
}



int main(int argc, char **argv)
{
    // sys init
    srand(time(NULL));
    cout << PCL_VERSION_PRETTY << endl;
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
        
        scan_combine_stl(FILELIST, NUM_SAMPLE);
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

