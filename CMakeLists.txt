CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(openpose-plus)

SET(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#SET(OpenCV_DIR /home/hks/opencv-3.4.1/build)
#FIND_PACKAGE(OpenCV REQUIRED )
FIND_PACKAGE(gflags)
FIND_PACKAGE(Threads REQUIRED)

#source directory
aux_source_directory(src SRC_LISTS)

#head file path
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/cuda-9.0)
include_directories(${PROJECT_SOURCE_DIR}/include/tensorRT)
include_directories(${PROJECT_SOURCE_DIR}/include/3rdparty)
include_directories(/usr/include/x86-64-linux-gnu)
include_directories(/usr/include)

#library
link_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/lib/cuda-9.0)
link_directories(${PROJECT_SOURCE_DIR}/lib/tensorRT)
link_directories(${PROJECT_SOURCE_DIR}/lib/opencv)
link_directories(/usr/lib/x86-64-linux-gnu)


add_executable(demo_batch_detector src/demo_batch_detector.cpp)#${SRC_LISTS}

target_link_libraries(demo_batch_detector 
                      stream-detetor.a 
                      pose-detetor.a 
                      openpose-plus.a 
                      helpers.a 
                      opencv_core
                      opencv_imgproc
                      opencv_highgui
                      opencv_imgcodecs
                      opencv_videoio 
                      Threads::Threads 
                      cudart 
                      cudnn
                      nvinfer
                      nvparsers
                      gflags)

add_executable(demo_stream_detector src/demo_stream_detector.cpp)

target_link_libraries(demo_stream_detector 
                      stream-detetor.a 
                      pose-detetor.a 
                      openpose-plus.a 
                      helpers.a 
                      opencv_core
                      opencv_imgproc
                      opencv_highgui
                      opencv_imgcodecs
                      opencv_videoio 
                      Threads::Threads 
                      cudart 
                      cudnn
                      nvinfer
                      nvparsers
                      gflags)

add_executable(demo_live_camera src/demo_live_camera.cpp)

target_link_libraries(demo_live_camera 
                      stream-detetor.a 
                      pose-detetor.a 
                      openpose-plus.a 
                      helpers.a 
                      opencv_core
                      opencv_imgproc
                      opencv_highgui
                      opencv_imgcodecs
                      opencv_videoio 
                      Threads::Threads 
                      cudart 
                      cudnn
                      nvinfer
                      nvparsers
                      gflags)

