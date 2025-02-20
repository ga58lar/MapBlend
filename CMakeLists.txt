cmake_minimum_required(VERSION 3.22)
project(mapblend VERSION 0.0.1 LANGUAGES C CXX CUDA)

set(CMAKE_CUDA_ARCHITECTURES "80")

enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

add_subdirectory(thirdparty/OpenLiDARMap/cpp)
add_subdirectory(thirdparty/aw_lidar_centerpoint)

find_package(Eigen3 REQUIRED)
find_package(Iridescence REQUIRED)

# Add your library
add_library(mapblend_lib
    include/mapblend.hpp
    include/pipeline_extension.hpp
)

target_include_directories(mapblend_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mapblend_lib PUBLIC 
    Eigen3::Eigen
    openlidarmap_lib
    centerpoint_lib
    centerpoint_cuda_lib
    tensorrt_common
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    yaml-cpp
    Eigen3::Eigen
    stdc++fs
)

# Add executable
add_executable(mapblend main.cpp)
target_link_libraries(mapblend PRIVATE 
    mapblend_lib
    openlidarmap_lib
    centerpoint_lib
    centerpoint_cuda_lib
    tensorrt_common
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    yaml-cpp
    Eigen3::Eigen
    stdc++fs
    Iridescence::iridescence
)