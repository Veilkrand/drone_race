cmake_minimum_required(VERSION 2.8.3)
project(spline_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  dynamic_reconfigure)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Planner.cfg
)

catkin_package(
        LIBRARIES eigen_spline
)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(eigen_spline src/eigen_spline.cpp)

target_link_libraries(eigen_spline ${Boost_LIBRARIES})

set_target_properties(eigen_spline PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

# catkin_install_python(PROGRAMS scripts/planner.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

