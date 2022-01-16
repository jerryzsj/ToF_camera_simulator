cmake_minimum_required(VERSION 3.0)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set the path for project includes
include_directories(${PROJECT_SOURCE_DIR}/include)

# There exist different versions of the assimp library for different 
# ubuntu distros. 
execute_process(COMMAND lsb_release -sc 
  OUTPUT_VARIABLE _distro OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro}")
message("Compiling for Ubuntu version ${_distro}")

find_package(Eigen3 REQUIRED)
message("Found Eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(CGAL REQUIRED)
# message("Found CGAL")
# include_directories(${CGAL_INCLUDE_DIR})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("Found OpenMP")
  include_directories(${OpenMP_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
endif(OPENMP_FOUND)

find_package(PCL REQUIRED)
if(PCL_FOUND)
  message("Found PCL")
  include_directories(${PCL_INCLUDE_DIRS})
  # in case your PCL installation is partt of ros you might need to add this include path
  # include_directories(${PCL_INCLUDE_DIRS} /opt/ros/<ros_distro>/include/)
  set(LIBS ${LIBS} ${PCL_LIBRARIES})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenPCL_CXX_FLAGS} -DHAVE_PCL")
endif(PCL_FOUND)


find_package(CGAL REQUIRED)
if(CGAL_FOUND)
  message("Found CGAL")
  include_directories(${CGAL_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${CGAL_LIBS})
endif(CGAL_FOUND)


add_executable(three_cam_D500 src/three_cam_D500.cpp)
target_link_libraries(three_cam_D500 CGAL assimp ${LIBS})

add_executable(full_cam_D500 src/full_cam_D500.cpp)
target_link_libraries(full_cam_D500 CGAL assimp ${LIBS})

add_executable(three_cam_D200 src/three_cam_D200.cpp)
target_link_libraries(three_cam_D200 CGAL assimp ${LIBS})

add_executable(full_cam_D200 src/full_cam_D200.cpp)
target_link_libraries(full_cam_D200 CGAL assimp ${LIBS})
