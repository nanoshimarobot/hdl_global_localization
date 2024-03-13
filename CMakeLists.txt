cmake_minimum_required(VERSION 3.5)
project(hdl_global_localization)

option(ENABLE_TEASER "Build with Teaser++" OFF)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp
  src/${PROJECT_NAME}/bbs/bbs_localization.cpp
  src/${PROJECT_NAME}/ransac/voxelset.cpp
  src/${PROJECT_NAME}/ransac/ransac_pose_estimation.cpp
  src/${PROJECT_NAME}/engines/global_localization_bbs.cpp
  src/${PROJECT_NAME}/engines/global_localization_fpfh_ransac.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "hdl_global_localization::GlobalLocalizationNode")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)

ament_auto_add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME}_component PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_component ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_compile_definitions(${PROJECT_NAME}_component PRIVATE ${PCL_DEFINITIONS})

ament_auto_package()