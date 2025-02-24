cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

find_package(Protobuf REQUIRED)
    if(NOT PROTOBUF_FOUND)
        message(FATAL_ERROR "Protobuf not found")
    endif()
include_directories(
    ${PROTOBUF_INCLUDE_DIR}
    )

file(GLOB PROTO_FILES ${PROJECT_SOURCE_DIR}/protobuf/poseInfo.proto)
message("PROTO_FILES: ${PROTO_FILES}")
foreach(proto_name ${PROTO_FILES})
      file(RELATIVE_PATH proto_rel_name ${PROJECT_SOURCE_DIR} ${proto_name})
      list(APPEND REL_PROTO_FILES ${proto_rel_name})
endforeach()
message("REL_PROTO_FILES: ${REL_PROTO_FILES}")

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS  ${PROJECT_SOURCE_DIR}/${REL_PROTO_FILES})
message("PROTO_SRCS: ${PROTO_SRCS}")
message("PROTO_HDRS: ${PROTO_HDRS}")
add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})
message("PROTOBUF_INCLUDE_DIR: ${PROTOBUF_INCLUDE_DIR}")
target_include_directories(proto_lib PUBLIC ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(proto_lib PRIVATE ${PROTOBUF_STATIC_LIBRARIES})

find_package(Threads)
find_library(HIREDIS_LIB hiredis)
find_library(REDIS_PLUS_PLUS_LIB redis++)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

link_libraries(${PROTOBUF_LIBRARIES})

# add_library(${PROJECT_NAME} 
#     ${PROTO_SRCS}
#     ${PROTO_HDRS}
#     )

target_link_libraries(
    ${PROJECT_NAME}
    ${PROTOBUF_LIBRARY}
    )



# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Redis)

add_executable(mono_tum
Examples/Redis/mono_tum.cc)
target_link_libraries(mono_tum ${REDIS_PLUS_PLUS_LIB} pthread ${HIREDIS_LIB})
target_link_libraries(mono_tum ${PROJECT_NAME})
target_link_libraries(mono_tum PUBLIC ${PROJECT_NAME} PRIVATE proto_lib)

