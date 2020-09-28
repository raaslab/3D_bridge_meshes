cmake_minimum_required(VERSION 2.8)
project(octomap_server)
add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME} src/OctomapServer.cpp src/OctomapServerTrimmed.cpp src/OctomapServerMultilayer.cpp src/TrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_library(${PROJECT_NAME}_color src/OctomapServer.cpp src/OctomapServerTrimmed.cpp src/OctomapServerMultilayer.cpp src/TrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME}_color ${LINK_LIBS})
add_dependencies(${PROJECT_NAME}_color ${PROJECT_NAME}_gencfg)
target_compile_definitions(${PROJECT_NAME}_color PUBLIC COLOR_OCTOMAP_SERVER)

add_executable(octomap_server_node src/octomap_server_node.cpp)
target_link_libraries(octomap_server_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_server_node_trimmed src/octomap_server_node_trimmed.cpp)
target_link_libraries(octomap_server_node_trimmed ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_color_server_node src/octomap_server_node.cpp)
target_link_libraries(octomap_color_server_node ${PROJECT_NAME}_color ${LINK_LIBS})

add_executable(octomap_server_static src/octomap_server_static.cpp)
target_link_libraries(octomap_server_static ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_server_multilayer src/octomap_server_multilayer.cpp)
target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_saver src/octomap_saver.cpp)
target_link_libraries(octomap_saver ${PROJECT_NAME} ${LINK_LIBS})

add_executable(octomap_tracking_server_node src/octomap_tracking_server_node.cpp)
target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS})

# Nodelet
add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

add_library(octomap_color_server_nodelet src/octomap_server_nodelet.cpp)
target_link_libraries(octomap_color_server_nodelet ${PROJECT_NAME}_color ${LINK_LIBS})

add_executable(kevin src/kevin.cpp)
target_link_libraries(kevin ${PROJECT_NAME} ${LINK_LIBS})

add_executable(zFilter src/zFilter.cpp)
target_link_libraries(zFilter ${PROJECT_NAME} ${LINK_LIBS})

add_executable(sandBox src/sandBox.cpp)
target_link_libraries(sandBox ${PROJECT_NAME} ${LINK_LIBS})

add_executable(markers src/markers.cpp)
target_link_libraries(markers ${catkin_LIBRARIES})

add_executable(visitedPoints_Publisher src/visitedPoints_Publisher.cpp)
target_link_libraries(visitedPoints_Publisher ${PROJECT_NAME} ${LINK_LIBS})

# add_executable(hector_explorer src/Explorer.cpp src/explorer_node.cpp)
# target_compile_options(hector_explorer PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})
# target_link_libraries(hector_explorer ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})
# add_dependencies(hector_explorer hector_uav_msgs_generate_messages hector_moveit_actions_generate_messages)

# install targets:
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_color
  octomap_server_node
  octomap_color_server_node
  octomap_server_static
  octomap_server_multilayer
  octomap_saver
  octomap_tracking_server_node
  octomap_server_nodelet
  octomap_color_server_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(FILES
  nodelet_plugins.xml
  color_nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
