cmake_minimum_required(VERSION 2.8.3)
project(example_hah)
include_directories(
../../sbgECom/src/
)
 
add_executable(example src/ellipseMinimal.c)

#link_directories(../../sbgECom/libSbgECom.a)
target_link_libraries(example "/home/jeffsan/workspace/Ellipse/Software Development/sbgECom/libSbgECom.a")

