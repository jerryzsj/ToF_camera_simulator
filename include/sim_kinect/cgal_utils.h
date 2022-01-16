#include <CGAL/config.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef K::Segment_3 Segment;
typedef K::Triangle_3 Triangle;

typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;

typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef AABB_triangle_traits::Point_and_primitive_id Point_and_Primitive_id;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef Tree::Object_and_primitive_id Object_and_Primitive_id;


void cgalVersion()
{
	std::cout << "My CGAL library is " <<  CGAL_VERSION_NR << " (1MMmmb1000)" << std::endl;
  // // std::cout << "where MM is the major number release, mm is the minor number release, and "
  // //           << "b is the bug fixing number release." << std::endl;
  std::cout << "***-------------------------***" << std::endl;
}