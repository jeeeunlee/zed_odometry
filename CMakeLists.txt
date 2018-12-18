cmake_minimum_required( VERSION 2.8 )
project( zed_odometry )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package( OpenCV 3.1 REQUIRED )



include_directories( 
    ${PROJECT_SOURCE_DIR}
    "include/zed_odometry"
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

add_library( zed_odometry_lib
    src/zed_frame.cpp
    src/zed_point3d.cpp
    src/zed_param.cpp
    src/optimizer.cpp
)


add_executable( zed_odometry src/zed_odometry.cpp  )

target_link_libraries( zed_odometry 
    zed_odometry_lib
    ${OpenCV_LIBS} 
)