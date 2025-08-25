#Just specifies the Versions required as well as name of the package
cmake_minimum_required(VERSION 3.5)
project(hi_decklink_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

# Not like this beacuse of missing includes
# Includes libdecklink
add_subdirectory(vendor/libdecklink)

# Declares dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_transport
        camera_info_manager
        #this is a ROS wrapper for openCV2 - both of them needed
        cv_bridge
        sensor_msgs
        )

###################################
## catkin specific configuration ##
###################################
#makes other packages be dependable on hi_decklink_ros
catkin_package()

###########
## Build ##
###########

# Include the catkin dirs as SYSTEM to suppress warnings 
include_directories(include/ SYSTEM ${catkin_INCLUDE_DIRS})

#Adds executable named publisher 
#both files are executed as a binary
add_executable(publisher src/publisher src/decklink_camera_driver.cpp)
#These are just the dependencies
target_link_libraries(publisher decklink ${catkin_LIBRARIES})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber decklink ${catkin_LIBRARIES})

add_executable(img2ros src/img2ros.cpp)
target_link_libraries(img2ros decklink ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# installs publisher and subscriber into standard ROS binary locations
install(TARGETS publisher subscriber
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#installs .launch files into the package share folder
#can thus be run with roslaunch
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        )

# Dummy target for CLion
# Asks to look for all header files and adds all headers as sources
FILE (GLOB_RECURSE clion_all_headers
        ${CMAKE_SOURCE_DIR}/**/*.hpp
        ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(all_headers SOURCES ${clion_all_headers})
