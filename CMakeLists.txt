cmake_minimum_required(VERSION 3.0)
project(gazebo_wind_field)
find_package(gazebo REQUIRED)

###########################
### Include directories ###
###########################
include_directories(${GAZEBO_INCLUDE_DIRS})

########################
### Link Directories ###
########################
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

##############################################################################################
### Add a library target called gazebo_wind_field to be built from the source files listed ###
##############################################################################################
add_library(gazebo_wind_field SHARED WindPlugin.cc) # add a library to the project using specified source files
target_include_directories(
    gazebo_wind_field 
    PRIVATE ${CMAKE_SOURCE_DIR}/include
    )
target_link_libraries(gazebo_wind_field ${GAZEBO_LIBRARIES})


