#Just specifies the Versions required as well as name of the package
cmake_minimum_required(VERSION 3.5)
project(hi_decklink_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# Not like this beacuse of missing includes
# Includes libdecklink
add_subdirectory(vendor/libdecklink)

# Declares dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
        

###################################
## catkin specific configuration ##
###################################
###########
## Build ##
###########

# Include the headers for decklink - necessary for compiling decklink
target_include_directories(decklink
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/vendor/libdecklink/include"
)

#Adds executable named publisher 
#both files are executed as a binary
add_executable(publisher src/publisher.cpp src/decklink_camera_driver.cpp)
#These are just the dependencies
target_include_directories(publisher PRIVATE include)
target_link_libraries(publisher decklink)
ament_target_dependencies(publisher rclcpp image_transport camera_info_manager cv_bridge sensor_msgs std_srvs)


add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber decklink)
ament_target_dependencies(subscriber rclcpp image_transport camera_info_manager cv_bridge sensor_msgs)


add_executable(img2ros src/img2ros.cpp)
target_link_libraries(img2ros decklink)
ament_target_dependencies(img2ros rclcpp image_transport camera_info_manager cv_bridge sensor_msgs)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# installs publisher and subscriber into standard ROS binary locations
install(TARGETS img2ros publisher subscriber
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS decklink
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#installs .launch files into the package share folder
#can thus be run with roslaunch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.xml")

# Dummy target for CLion
# Asks to look for all header files and adds all headers as sources
FILE (GLOB_RECURSE clion_all_headers
        ${CMAKE_SOURCE_DIR}/**/*.hpp
        ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(all_headers SOURCES ${clion_all_headers})

#makes other packages be dependable on hi_decklink_ros
ament_export_include_directories(include)
#shows the dependencies
ament_export_dependencies(rclcpp image_transport camera_info_manager cv_bridge sensor_msgs)

#installs package.xml, registers it and installs configuration files for CMake
#makes other packages be dependable on hi_decklink_ros
ament_package()

