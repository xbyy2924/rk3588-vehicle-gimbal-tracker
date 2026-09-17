cmake_minimum_required(VERSION 3.16)

project(rk3588_gimbal_linux_client VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)
find_path(SDL2_INCLUDE_DIR NAMES SDL2/SDL.h REQUIRED)
find_library(SDL2_LIBRARY NAMES SDL2 REQUIRED)

set(RKNN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknpu2")
set(RKNN_LIBRARY "${RKNN_ROOT}/Linux/aarch64/librknnrt.so")
if(NOT EXISTS "${RKNN_LIBRARY}")
    message(FATAL_ERROR "Missing RKNN runtime: ${RKNN_LIBRARY}")
endif()

add_library(rknnrt SHARED IMPORTED GLOBAL)
set_target_properties(rknnrt PROPERTIES IMPORTED_LOCATION "${RKNN_LIBRARY}")

add_executable(app
    src/app.cc
    src/gimbal_udp_controller.cpp
    src/gimbal_command_console.cpp
    src/yolov8.cc
    src/postprocess.cc
    src/image_utils.c
    src/file_utils.c
)

target_include_directories(app PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${RKNN_ROOT}/include"
    "${SDL2_INCLUDE_DIR}"
)

target_compile_options(app PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wshadow>"
    "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wshadow>"
)

target_link_libraries(app PRIVATE
    rknnrt
    "${SDL2_LIBRARY}"
    Threads::Threads
    dl
    m
)

set_target_properties(app PROPERTIES
    BUILD_RPATH "${RKNN_ROOT}/Linux/aarch64"
    INSTALL_RPATH "$ORIGIN/lib"
)

install(TARGETS app RUNTIME DESTINATION .)
install(FILES "${RKNN_LIBRARY}" DESTINATION lib)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/model/coco_80_labels_list.txt" DESTINATION model)

set(RKNN_MODEL "${CMAKE_CURRENT_SOURCE_DIR}/model/yolov8n_rk3588_int8.rknn")
if(EXISTS "${RKNN_MODEL}")
    install(FILES "${RKNN_MODEL}" DESTINATION model)
else()
    message(WARNING
        "RKNN model is not in source model/. The program will build, but copy "
        "yolov8n_rk3588_int8.rknn there before running ./app.")
endif()
