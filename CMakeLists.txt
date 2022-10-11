cmake_minimum_required(VERSION 3.0.2)
project(motion)

find_package(catkin REQUIRED COMPONENTS 
  utils
)

catkin_package(
  INCLUDE_DIRS include  
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib 
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} utils_lib)