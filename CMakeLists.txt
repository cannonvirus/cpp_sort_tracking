cmake_minimum_required(VERSION 3.5.1)
project("sort-cpp" VERSION 1.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set (CUDA_PROPAGATE_HOST_FLAGS ON)
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} nvcc -Wno-deprecated-gpu-targets -arch=sm_35 -rdc=true -std=c++11 -Xcompiler -mcmodel=medium" )

# 형태
# 폴더 필요 include, src, bin
# include <- *.h
# src <- *.cpp (실행문 돌리면 .o 파일도 생성됨)
# bin <- 원래는 실행파일 나오는 곳 // main함수 있으면 사용

# Try to find OpenCV =============================================================================
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else ()
    message(FATAL_ERROR "Could not locate OpenCV")
endif()
# Try to find OpenCV =============================================================================

# find_package(CUDA 11.1 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
# include_directories(${CUDA_INCLUDE_DIRS})
# link_directories(${CUDA_LIBRARY_DIRS})

file(GLOB SOURCE_FILES src/*.cpp)

## .so 파일로 만드는 코드 main.cpp 빼고 사용 // error 나지만, .so파일은 생성됨.
# add_library(tracker SHARED ${SOURCE_FILES})

## .a 파일로 만드는 코드
# add_library(tracker2 STATIC ${SOURCE_FILES})


# 실행파일 만드는 코드 main 함수가 필수
set(OUTPUT_EXEC "${CMAKE_PROJECT_NAME}_app")
add_executable(${OUTPUT_EXEC} ${SOURCE_FILES})
target_link_libraries (
        ${OUTPUT_EXEC}
        ${OpenCV_LIBS}
)


# tracker 옆에 {OUTPUT_EXEC} 항목 추가 (,) 안써야 함
# install(TARGETS tracker
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         PUBLIC_HEADER DESTINATION include
#         )