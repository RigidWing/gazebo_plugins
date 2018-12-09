cmake_minimum_required(VERSION 3.0)
#project(gazebo_wind_field)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)


###############################
##### Protobuf messages #######
###############################
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  ${CMAKE_SOURCE_DIR}/msgs/WindField.proto
  ${CMAKE_SOURCE_DIR}/msgs/time.proto
  ${CMAKE_SOURCE_DIR}/msgs/vector3d.proto
)

message("The messages" ${msgs})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
message(The PROTO_SRCS is at ${PROTO_SRCS})
message(The PROTO_HDRS is at ${PROTO_HDRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


##############################################
########## Include and link directories ######
##############################################
include_directories(${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
include_directories(${CMAKE_BINARY_DIR})

# The following include directory points to the directory that contains the latest gazebo source code obtained from bitbucket
#set(PATH_TO_GAZEBO_SRC_DIR "/home/isabelle/gazebo_from_bitbucket")
#message("The path to the gazebo src dir", ${PATH_TO_GAZEBO_SRC_DIR})
#include_directories(${PATH_TO_GAZEBO_SRC_DIR})
# Get the corresponding paths for the cc files included in the directory.

########################
### Link Directories ###
########################
link_directories(${GAZEBO_LIBRARY_DIRS} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
include_directories(include)

##################################################################################
add_library(gazebo_wind_field SHARED gazebo_wind_field_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_wind_field
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    )

target_include_directories(
    gazebo_wind_field
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_wind_field ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_wind_field ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_lift_drag SHARED liftdrag_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_lift_drag
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_lift_drag ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_lift_drag ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_lift_distribution_plugin SHARED lift_distribution_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_lift_distribution_plugin
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_lift_distribution_plugin ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_lift_distribution_plugin	 ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_lift_drag_with_lookup SHARED liftdrag_with_lookup_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_lift_drag_with_lookup
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_lift_drag_with_lookup ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_lift_drag_with_lookup	 ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_visualize_vector_plugin SHARED visualize_vector_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_visualize_vector_plugin
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_visualize_vector_plugin ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_visualize_vector_plugin	 ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_control_plugin SHARED control_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_control_plugin
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_control_plugin ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_control_plugin	 ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_verify_liftdrag_plugin SHARED verify_liftdrag_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_verify_liftdrag_plugin
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_verify_liftdrag_plugin ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_verify_liftdrag_plugin	 ${PROTOBUF_LIBRARY})

##################################################################################
add_library(gazebo_control_input_plugin SHARED control_input_plugin.cpp ${PROTO_SRCS})
target_include_directories(
    gazebo_control_input_plugin
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
    )

target_link_libraries(gazebo_control_input_plugin ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${PROTOBUF_LIBRARY})

##################################################################################
#set (headers
#  qt.h
#  qt_test.h)

#add_definitions(${Qt5Core_DEFINITIONS})


#add_library(gazebo_gui_plugin SHARED CessnaGUIPlugin.cc)
#target_include_directories(
#    gazebo_gui_plugin
#    PRIVATE ${CMAKE_BINARY_DIR}
#    ${Qt5Core_INCLUDE_DIRS}
#    )

#target_link_libraries(gazebo_gui_plugin ${GAZEBO_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} gzqtpropertybrowser)
#target_link_libraries(gazebo_gui_plugin	 ${PROTOBUF_LIBRARY})


##################################################################################

add_executable(send_commandline_protobuf send_protobuf_msgs.cc ${PROTO_SRCS})
target_include_directories(
    send_commandline_protobuf
    PRIVATE ${CMAKE_SOURCE_DIR}/msgs
    PRIVATE ${CMAKE_BINARY_DIR}
)

target_link_libraries(send_commandline_protobuf ${GAZEBO_LIBRARIES})
target_link_libraries(send_commandline_protobuf ${PROTOBUF_LIBRARY})

##################################################################################
