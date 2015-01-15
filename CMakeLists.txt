cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILT_TYPE RelWithDebInfo)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_check_for_sse()

add_definitions(-Wall -Wno-missing-field-initializers -fopenmp)

#####################################################################
# PE library
find_package(Eigen)
find_package(OpenCV)
find_package(PCL)
FIND_PACKAGE(vikit_common REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)


include_directories(SYSTEM ${Eigen_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
                           ${vikit_common_INCLUDE_DIRS}
                           ${Sophus_INCLUDE_DIRS}
)
#rosbuild_add_library(posest src/pe.cpp src/howardMatcher.cpp src/peh.cpp src/pe3d.cpp src/planarSFM.cpp src/pe2d.cpp src/pnp_ransac.cpp)
#rosbuild_add_compile_flags(posest ${SSE_FLAGS})
#target_link_libraries(posest tbb gomp ${OpenCV_LIBS} ${PCL_LIBS}) # Intel Thread Building Blocks

#####################################################################
# PE test library: artificial data generation
#rosbuild_add_library(pe_simulated test/simulated.cpp)
#rosbuild_add_compile_flags(pe_simulated ${SSE_FLAGS})
#target_link_libraries(pe_simulated posest gomp)

#####################################################################
# unit tests
#



######################################################################
# executables
#

# test pose estimation
#rosbuild_add_executable(test/run_posest test/run_posest.cpp)
#target_link_libraries(test/run_posest posest gomp ${OpenCV_LIBS})

# test pose estimation for mono camera
#rosbuild_add_executable(test/run_simulated2d test/run_simulated2d.cpp)
#target_link_libraries(test/run_simulated2d pe_simulated gomp)

# read sequence and SBA it
#rosbuild_add_executable(test/run_sequence test/run_sequence.cpp)
#target_link_libraries(test/run_sequence posest gomp)



######################################################################
# visualization
#

######################################################################
# ESM tracking
#

#rosbuild_add_library(esm src/esm.cpp)
#rosbuild_add_compile_flags(esm ${SSE_FLAGS})
#target_link_libraries(esm ${OpenCV_LIBRARIES})

#rosbuild_add_executable(test/run_esm test/run_esm.cpp)
#target_link_libraries(test/run_esm esm gomp)

rosbuild_add_library(psm src/planarSFM.cpp src/pnp_ransac.cpp)
rosbuild_add_compile_flags(psm ${SSE_FLAGS})
target_link_libraries(psm tbb ${OpenCV_LIBRARIES})


#rosbuild_add_library(psm1 src/planarSFM_new.cpp)
#rosbuild_add_compile_flags(psm1 ${SSE_FLAGS})
#target_link_libraries(psm1 sba_ros posest ${OpenCV_LIBRARIES} )
#target_link_libraries(psm1 ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ) 

rosbuild_add_executable(test/run_psm src/newSFM.cpp)
target_link_libraries(test/run_psm posest psm ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )

rosbuild_add_executable(test/plane src/planeseg.cpp)
target_link_libraries(test/plane psm ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )


rosbuild_add_executable(test/plane_sfm src/plane_sarthak.cpp )
target_link_libraries(test/plane_sfm psm ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${vikit_common_LIBRARIES} ${Sophus_LIBRARIES})

#rosbuild_add_executable(test/run_pcl src/pcl.cpp)
#target_link_libraries(test/run_pcl ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )
