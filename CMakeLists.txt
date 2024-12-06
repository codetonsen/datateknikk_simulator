cmake_minimum_required(VERSION 3.17)
project(New_Version_Simulator)

set(CMAKE_CXX_STANDARD 20)

# CHATGPT FIX for some error when building pcl and g2o
add_definitions(-D_WIN32_WINNT=0x0A00)
set(CMAKE_VERBOSE_MAKEFILE ON)
if(MSVC)
    add_compile_options(/bigobj)
endif()

# FetchContent for external dependencies
include(FetchContent)

# Fetch threepp from GitHub
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG master
)

set(SIMPLE_SOCKET_BUILD_TESTS OFF)
FetchContent_Declare(
        SimpleSocket
        GIT_REPOSITORY https://github.com/markaren/SimpleSocket.git
        GIT_TAG master
)

# Make available
FetchContent_MakeAvailable(SimpleSocket threepp)

# Add executable
add_executable(New_Version_Simulator
        main.cpp
        source/Slam.cpp
        source/Sphero.cpp
        source/SimulatorUtilities.cpp
        source/KeyHandler.cpp
        source/Grid.cpp
)


find_package(VTK REQUIRED)


find_package(PCL CONFIG REQUIRED COMPONENTS
        visualization
        common
        io
        registration
)
find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o CONFIG REQUIRED COMPONENTS
        core
        stuff
        types_icp
        types_slam3d
        solver_csparse
        solver_dense
        csparse_extension
        viewer
)

# Include folder
target_include_directories(New_Version_Simulator PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${G2O_INCLUDE_DIRS}
        ${pcl_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
)
# include spesific links
target_link_libraries(New_Version_Simulator PRIVATE
        g2o::core
        g2o::stuff
        g2o::types_slam2d
        threepp
        simple_socket
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}

)

# Copy assets folder to the build directory
add_custom_command(
        TARGET New_Version_Simulator POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/Assets $<TARGET_FILE_DIR:New_Version_Simulator>/Assets
)
