cmake_minimum_required(VERSION 3.16)
# project(v1)

# find_package(k4a REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

# set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)


# include_directories(.)
# include_directories( 
#     ${PCL_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )
# add_executable(v1 src/camera.cpp src/pointcloud.cpp)

# link_directories(
#     ${PCL_LIBRARY_DIRS} 
#     )
# target_link_libraries(v1 k4a::k4a)
# target_link_libraries(v1 
#     ${OpenCV_LIBS}   
#     ${PCL_LIBRARIES}
    
# )
# 查看点云
project(v2)

find_package(k4a REQUIRED)
find_package(PCL REQUIRED)

include_directories(.)
include_directories( 
    ${PCL_INCLUDE_DIRS}
)
link_directories(
    ${PCL_LIBRARY_DIRS} 
    )
add_executable(v2 src/seePLY.cpp)
target_link_libraries(v2 k4a::k4a)
target_link_libraries(v2 
    ${PCL_LIBRARIES}
)
