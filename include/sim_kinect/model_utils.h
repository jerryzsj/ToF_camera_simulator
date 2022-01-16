#ifndef MODEL_UTILS_H_
#define MODEL_UTILS_H_

#include <iostream>
#include <string>
#include <list>
#include "sim_kinect/std_utils.h"
#include "sim_kinect/eigen_utils.h"

#ifdef HAVE_precise
#include "assimp/assimp.h"
#include "assimp/aiPostProcess.h"
#include "assimp/aiScene.h"
#elif defined HAVE_quantal // uses assimp3.0 
#include "assimp/cimport.h"
#include "assimp/cexport.h"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#else
#include "assimp/cimport.h"
#include "assimp/cexport.h"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#endif

#include <Eigen/Dense>
#include <Eigen/Core>
// #include <Eigen/Geometry>

#include "boost/algorithm/string.hpp"

#include <CGAL/config.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/number_utils.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Kernel_traits.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/aff_transformation_tags.h>

// using namespace std;
// using namespace Eigen;


typedef float FT;
typedef CGAL::Simple_cartesian<FT> K;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef K::Segment_3 Segment;
typedef K::Triangle_3 Triangle;

typedef CGAL::Aff_transformation_3<K> AffTransform;

typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;

typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef AABB_triangle_traits::Point_and_primitive_id Point_and_Primitive_id;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef Tree::Object_and_primitive_id Object_and_Primitive_id;

struct TreeAndTri {
  std::vector<K::Triangle_3> triangles;
  std::vector<K::Point_3> points;
  std::vector<Eigen::RowVector3i> indices;
  std::vector<int> part_ids;
  Tree tree;
  AffTransform transform;
  Eigen::Matrix4f eigen_transform;
};

struct MeshAndModel
{
	// vector stored the imported meshes
  std::vector<aiMesh*> myMeshes;
  // vertices from stl file
  std::vector<Eigen::MatrixXf> myVertices;
  // number of vertices
  std::vector<int> myNumVertices;
  // number of faces
  std::vector<int> myNumFaces;
  // transformation matrix for model
  Eigen::Affine3f myTransform;

  MeshAndModel()
  {
  	myTransform.Eigen::Affine3f::Identity();
  }
};


// Init tree
void initTree(TreeAndTri* search)
{
	search->tree.rebuild(search->triangles.begin(), search->triangles.end());
  	search->tree.accelerate_distance_queries();
	// std::cout << "inited tree" << std::endl;
  // std::cout << INTERVAL;
  return;
}


// Upload the corresponding label to the tree which is associated to the face
void initPartIDs(TreeAndTri* search, MeshAndModel* model)
{
  for (unsigned i = 0; i < model->myMeshes.size(); i++)
  {
  	const struct aiMesh* mesh = model->myMeshes[i];
	  for (unsigned f=0; f < model->myNumFaces[i]; f++)
		{
			search->part_ids.push_back(i);
		}
	}
	// std::cout << "inited " << search->part_ids.size() << " tree part_ids" << std::endl;
	return;
}


// init the triangle indices to the search tree
// this would only need to be done ones in the beginning
void initTreeFaces(TreeAndTri* search, MeshAndModel* model)
{
  	// search->triangles.resize(numFaces_);
  	int max_indice = 0;
  	for (unsigned i = 0; i < model->myMeshes.size(); i++)
  	{
  		const struct aiMesh* mesh = model->myMeshes[i];
  		for (unsigned f=0; f < model->myNumFaces[i]; f++)
		{
			const struct aiFace* face_ai = &mesh->mFaces[f];
			// std::cout << face_ai->mNumIndices << std::endl;
			// std::cout << f << std::endl;
			if(face_ai->mNumIndices!=3)
			{
	 	 		std::cerr << "not a triangle!: " << face_ai->mNumIndices << " vertices" << std::endl;
	 	 		throw;
	 	 	}

			// faces contain the original vert indices; they should be offset by the verts
			// of the previously processed meshes when one part can have multiple meshes!
			search->triangles.push_back(K::Triangle_3(search->points[face_ai->mIndices[0]+max_indice], 
				       search->points[face_ai->mIndices[1]+max_indice],
				       search->points[face_ai->mIndices[2]+max_indice]));
			search->indices.push_back(Eigen::RowVector3i(face_ai->mIndices[0]+max_indice, 
				      face_ai->mIndices[1]+max_indice, face_ai->mIndices[2]+max_indice));
			// std::cout << face_ai->mIndices[0] << "  ";
			// std::cout << face_ai->mIndices[1] << "  ";
			// std::cout << face_ai->mIndices[2] << "  ";
			// std::cout << std::endl;
		}
		max_indice += model->myMeshes[i]->mNumVertices;
  	}
 	 // std::cout << "inited " << search->triangles.size() << " tree faces" << std::endl;
  	return;
}


// init vertices in the search tree
void initTreeVertices(TreeAndTri* search, MeshAndModel* model)
{
		float x_, y_, z_;
		x_ = y_ = z_=0.0;
  	for (int i = 0; i < model->myMeshes.size(); ++i)
  	{
  		for(int v=0; v < model->myNumVertices[i]; ++v)
			{
				x_+=model->myVertices[i](0,v);
				y_+=model->myVertices[i](1,v);
				z_+=model->myVertices[i](2,v);
			}
	  }
	 	x_ = x_ / float(model->myMeshes.size());
	 	y_ = y_ / float(model->myMeshes.size());
	 	z_ = z_ / float(model->myMeshes.size());
	 	
	 	for (int i = 0; i < model->myMeshes.size(); ++i)
  	{
  		for(int v=0; v < model->myNumVertices[i]; ++v)
			search->points.push_back(K::Point_3(model->myVertices[i](0,v)-x_,
			       model->myVertices[i](1,v)-y_,
			       model->myVertices[i](2,v)-z_));
		}
  	// std::cout << "inited tree vertices" << std::endl;
  	return;
}


// update tree faces with transformed new points
// with same indices
void updateTreeFaces(TreeAndTri* search)
{
	for(int i = 0; i<search->indices.size();i++)
	{
		search->triangles[i] = K::Triangle_3(search->points[search->indices[i].x()], 
				       search->points[search->indices[i].y()],
				       search->points[search->indices[i].z()]);
	}
}


void updateTreeTransform(TreeAndTri* search)
{
	for (int i =0; i< search->points.size(); i++)
	{
		search->points[i] = search-> transform(search->points[i]);
	}
	updateTreeFaces(search);
	search->tree.rebuild(search->triangles.begin(), search->triangles.end());
  	search->tree.accelerate_distance_queries();
	return;
}


void setTreeCoorOrigin(TreeAndTri* search)
{
	K::Iso_cuboid_3 c3 = CGAL::bounding_box(search->points.begin(), search->points.end());
	Vector translate = Vector( -(c3.xmin()+c3.xmax())/2.0,-(c3.ymin()+c3.ymax())/2.0,-c3.zmin()+10.0);
	// Vector translate = Vector(-c3.xmin(),-c3.ymin(),-c3.zmin() + 10);
	// cout << Vector(-c3.xmin(),-c3.ymin(),-c3.zmin()) << endl;
	search->transform = AffTransform(CGAL::TRANSLATION, translate); // = CGAL::Aff_transformation_3<K> ;
	// cout << search->transform.m(0,3) << endl;
	// cout << search->transform.m(1,3) << endl;
	// cout << search->transform.m(2,3) << endl;
	// cout << search->transform.m(3,3) << endl;
	updateTreeTransform(search);
	// cout << c3 << endl;
	return;
}

void randTreeTransform(TreeAndTri* search)
{
	Eigen::Matrix4f eigen_rotate = eigen_rand_affrotate();
	// cout << "rand tree1"<<endl;

	// Eigen::Matrix4f eigen_translate = eigen_rand_afftranslate(0, 0, 0);
	// cout << "rand tree2"<<endl;

	Eigen::Matrix4f eigen_transform = eigen_rotate;
	// cout << eigen_transform.matrix();
	// cout << "rand tree3"<<endl;

	// cout << search->eigen_transform.matrix();
	// search->eigen_transform = eigen_transform;
	// cout << search->eigen_transform.matrix();
	// cout << "rand tree4"<<endl;

	search->transform = AffTransform(eigen_transform(0,0), eigen_transform(0,1),eigen_transform(0,2),eigen_transform(0,3),
							eigen_transform(1,0), eigen_transform(1,1),eigen_transform(1,2),eigen_transform(1,3),
							eigen_transform(2,0), eigen_transform(2,1),eigen_transform(2,2),eigen_transform(2,3));
	// cout << search->transform.m(0,3) << endl;
	// cout << search->transform.m(1,3) << endl;
	// cout << search->transform.m(2,3) << endl;
	// cout << search->transform.m(3,3) << endl;
	updateTreeTransform(search);
	return;
}


// init transformation matrix
void initTransform(MeshAndModel* model)
{
	model->myTransform.Eigen::Affine3f::Identity();
	// std::cout << "inited transform" << std::endl;
	return;
}

// init number of faces in MeshAndModel
void initNumFaces(MeshAndModel* model)
{
	std::vector<int> numFaces_;
	for (int i = 0; i < model->myMeshes.size(); ++i)
	{
		numFaces_.push_back(model->myMeshes[i]->mNumFaces);
	}
	model->myNumFaces = numFaces_;
	// std::cout << "inited number of faces" << std::endl;
	return;
}

// init number of vertices in MeshAndModel
void initNumVertices(MeshAndModel* model)
{
	std::vector<int> numVertices_;
	for (int i = 0; i < model->myMeshes.size(); ++i)
	{
		numVertices_.push_back(model->myMeshes[i]->mNumVertices);
	}
	// std::cout << "inited number of vertices" << std::endl;
	model->myNumVertices = numVertices_;
	return;
}


void initMeshVertices(std::vector<Eigen::MatrixXf> &myVertices, std::vector<aiMesh*> myMesh)
{
	// std::cout << "init mesh vertices:" << std::endl;
	for (unsigned m=0; m<myMesh.size(); m++)
	{
		//std::cout << m << std::endl;
		Eigen::MatrixXf vertices;
		vertices.resize(4, myMesh[m]->mNumVertices);
		for (unsigned v = 0; v < myMesh[m]->mNumVertices; v++)
		{
			vertices(0,v) = myMesh[m]->mVertices[v].x;
			vertices(1,v) = myMesh[m]->mVertices[v].y;
			vertices(2,v) = myMesh[m]->mVertices[v].z;
			vertices(3,v) = 1;
		}
		// std::cout << "init vertices for mesh " << m << std::endl;
		myVertices.push_back(vertices);
		// std::cout << myVertices[m](0,100) << std::endl;
		// std::cout << myVertices[m](1,100) << std::endl;
		// std::cout << myVertices[m](2,100) << std::endl;
		// std::cout << myVertices[m](3,100) << std::endl;
	}
	// std::cout << INTERVAL;
	return;
}


void loadMeshModel(std::vector<aiMesh*> &myMesh, string filePath)
{
	const struct aiScene* scene_;
  // aiScene constructor based on assimp lib
	// std::cout << "loading mesh model from files:" << std::endl;
	vector<string> myPath;
  // vector stored the paths
  boost::split(myPath, filePath, [](char c){return c == ' ';});
  // split the files string
  int numFiles = myPath.size();
  // number of files
  for (int i = 0; i < numFiles; i++)
	{
		// std::cout << "	from path->" << myPath[i] << std::endl;
    scene_ = aiImportFile(myPath[i].c_str(), aiProcessPreset_TargetRealtime_Quality);
    // init scene with stl file from file
    myMesh.push_back(scene_->mMeshes[0]);
    myMesh[myMesh.size()-1]->mName = "mesh" + to_string(myMesh.size()-1);
    // get mesh from scene_
    // numFaces_ += myMesh[i]->mNumFaces;
    // get number of faces for Mesh.No.i
    // numVertices_ += myMesh[i]->mNumVertices;
    // get number of vertices for Mesh.No.i
    // aiReleaseImport(scene_);
     std::cout << (myMesh[myMesh.size()-1]->mName.C_Str())
     << " with " << myMesh[myMesh.size()-1]->mNumFaces 
     << " faces and " << myMesh[myMesh.size()-1]->mNumVertices 
     << " vertices" << std::endl;
	}
  	// std::cout << INTERVAL;
    //aiReleaseImport(scene_);
  return;
}

void loadMeshSingle(std::vector<aiMesh*> &myMesh, string filePath)
{
	const struct aiScene* scene_;
  
  scene_ = aiImportFile(filePath.c_str(), aiProcessPreset_TargetRealtime_Quality);
  myMesh.push_back(scene_->mMeshes[0]);
  myMesh[myMesh.size()-1]->mName = "mesh" + to_string(myMesh.size()-1);
  
  std::cout << (myMesh[myMesh.size()-1]->mName.C_Str())
   << " with " << myMesh[myMesh.size()-1]->mNumFaces 
   << " faces and " << myMesh[myMesh.size()-1]->mNumVertices 
   << " vertices" << std::endl;
  return;
}


void loadMeshList(std::vector<aiMesh*> &myMesh, std::vector<string> &filePath)
{
	const struct aiScene* scene_;
	
  int numFiles = filePath.size();
  for (int i = 0; i < numFiles; i++)
	{
    scene_ = aiImportFile(filePath[i].c_str(), aiProcessPreset_TargetRealtime_Quality);
    myMesh.push_back(scene_->mMeshes[0]);
    myMesh[myMesh.size()-1]->mName = "mesh" + to_string(myMesh.size()-1);
    
     std::cout << (myMesh[myMesh.size()-1]->mName.C_Str())
     << " with " << myMesh[myMesh.size()-1]->mNumFaces 
     << " faces and " << myMesh[myMesh.size()-1]->mNumVertices 
     << " vertices" << std::endl;
	}
  return;
}



void cgalVersion()
{
	std::cout << "My CGAL library is " <<  CGAL_VERSION_NR << " (1MMmmb1000)" << std::endl;
  // // std::std::cout << "where MM is the major number release, mm is the minor number release, and "
  // //           << "b is the bug fixing number release." << std::std::endl;
  std::cout << INTERVAL;
}

#endif