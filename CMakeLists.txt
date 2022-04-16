SET(execName Recorder)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(${execName})

SET(SRC_FOLDER src)
add_compile_options(-DROOT_DIR="${CMAKE_CURRENT_LIST_DIR}")
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
ADD_EXECUTABLE(${execName} ${SRC_FILES})

find_package(ZED 3 REQUIRED)

#SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda/bin/nvcc)
#SET(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

include_directories(SYSTEM ${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(${execName}
                        ${ZED_LIBRARIES}
	                    ${SPECIAL_OS_LIBS}
                        ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY} )

#if(COMMAND cmake_policy)
#	cmake_policy(SET CMP0003 OLD)
#	cmake_policy(SET CMP0015 OLD)
#endif(COMMAND cmake_policy)
#
#SET(EXECUTABLE_OUTPUT_PATH ".")
#
#SET(VERSION_REQ_CUDA "8.0")
#
#IF(WIN32) # Windows
#    if (CMAKE_CL_64) 
#        SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
#        SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
#        SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
#    else()
#        message(FATAL_ERROR "You've selected the 32bit version of ${CMAKE_GENERATOR}. \n Please delete the cache (file->Delete Cache) and use the 64bit version. (${CMAKE_GENERATOR} Win64)")
#    endif()
#ELSE() # Linux
#    find_package(ZED 2 REQUIRED)
#    SET(SPECIAL_OS_LIBS "pthread")
#ENDIF(WIN32)
#
#find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
#
#link_directories(${ZED_LIBRARY_DIR})
#link_directories(${CUDA_LIBRARY_DIRS})
#
#SET(SRC_FOLDER src)
#FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")  
#ADD_EXECUTABLE(${execName} ${SRC_FILES})
#
#add_definitions(-std=c++0x -g -O3)
#
## Add the required libraries for linking:
#


    # SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda/bin/nvcc)
    # SET(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
    # enable_language(CUDA)
    # include_directories(SYSTEM ${GLFW_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    
	# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{HOME}/.mrover/build_env/bin/)