cmake_minimum_required(VERSION 2.6)
PROJECT(vision C CXX)
#SET(OpenCV_DIR ~/Development/libraries/opencv/release)
find_package( OpenCV REQUIRED )

# enable warnings
ADD_DEFINITIONS( -Wall )

SET (vision_SOURCES
# cpp files
	main.cpp
    bgpattern.cpp
    detector.cpp
    RangeFinder.cpp
    Masks.cpp

# h files
    bgpattern.h
    detector.h
    defines.h
    RangeFinder.h
    Masks.h

)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
#include_directories("${PROJECT_BINARY_DIR}")

#LINK_DIRECTORIES(/usr/local/lib ${PROJECT_BINARY_DIR})
LINK_DIRECTORIES(/usr/local/lib)
ADD_EXECUTABLE(detect ${vision_SOURCES})
TARGET_LINK_LIBRARIES(detect ${OpenCV_LIBS}) 

