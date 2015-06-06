cmake_minimum_required(VERSION 2.6)
PROJECT(objectdetect C CXX)
#SET(OpenCV_DIR ~/Development/libraries/opencv/release)
find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )

# enable warnings
ADD_DEFINITIONS( -Wall )

SET (objectdetect_SOURCES
# cpp files
	main.cpp
    bgpattern.cpp
    serial.cpp
    detector.cpp
    brain.cpp
    Map.cpp
    Robot.cpp
    simulation.cpp
    RangeFinder.cpp
    Masks.cpp

# h files
    bgpattern.h
    serial.h
    detector.h
    defines.h
    brain.h
    Map.h
    Robot.h
    simulation.h
    RangeFinder.h
    Masks.h
)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
#include_directories("${PROJECT_BINARY_DIR}")

#LINK_DIRECTORIES(/usr/local/lib ${PROJECT_BINARY_DIR})
LINK_DIRECTORIES(/usr/local/lib)
ADD_EXECUTABLE(detect ${objectdetect_SOURCES})
TARGET_LINK_LIBRARIES(detect ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

