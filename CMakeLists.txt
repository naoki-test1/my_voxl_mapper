cmake_minimum_required(VERSION 3.3)
project(voxl-mapper)

find_package(Eigen3 REQUIRED)

# There is a bug with voxl-eigen3's version of Eigen
# where vectorization is causing issues for the smoother
# disable for now (Disabled on Aug 24 2022)
add_definitions(-DEIGEN_DONT_VECTORIZE)

if(PLATFORM MATCHES APQ8096)
	add_definitions(-DPLATFORM_APQ8096)
elseif(PLATFORM MATCHES QRB5165)
	add_definitions(-DPLATFORM_QRB5165)
endif()


set (ROOT ./server)

# James' standard list of cmake flags
set(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wno-double-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Wno-unused-function ${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=gnu++14 -Wall -Wextra -Wuninitialized -Wattributes \
	-Wunused-but-set-variable -Wdouble-promotion -Wno-missing-field-initializers \
	-Wmissing-declarations -Wno-unused-function ${CMAKE_CXX_FLAGS}")

# Optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
	set(LIB_INSTALL_DIR /usr/lib64)
else()
	set(LIB_INSTALL_DIR /usr/lib)
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(server/mav_comm/mav_msgs)
add_subdirectory(server/mav_comm/mav_planning_msgs)
add_subdirectory(server/mav_trajectory_generation)
add_subdirectory(server/mav_planning_common)
add_subdirectory(server/loco_planner)
add_subdirectory(server/voxblox_planning_common)
add_subdirectory(server/mav_path_smoothing)
add_subdirectory(server/voxl-mapper)
add_subdirectory(server/mav_local_planner)
