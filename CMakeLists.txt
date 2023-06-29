cmake_minimum_required(VERSION 3.10)
project(phdMatrix VERSION 0.1.0 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# have a common find dependency command
# include("cmake/kevinBuild.cmake")

# add_library(${PROJECT_NAME} SHARED matrix.cpp hello.cppm)

# add_subdirectory(src)


add_library(clib SHARED
    clib/fileio.c
)
# target_include_directories(clib PUBLIC include)

# set(LIBMATH phdMatrix)
add_library(mathlib SHARED
    mathlib/cArray.cpp
    mathlib/cBaseMath.cpp
    mathlib/cMatrix.cpp
    mathlib/cMLError.cpp
    mathlib/cQuaternion.cpp
    mathlib/cRand.cpp
    mathlib/cRK4.cpp
    mathlib/cVector.cpp
)

add_library(filterlib SHARED
    filterlib/cCDKF.cpp
    filterlib/cDKF.cpp
    filterlib/cKalmanFilter.cpp
)
target_include_directories(filterlib PUBLIC mathlib)
target_link_libraries(filterlib PUBLIC mathlib)

add_executable(nav
    nav/nav_filter.cpp
    nav/navigation.cpp
)
target_include_directories(nav PUBLIC mathlib filterlib clib)
target_link_libraries(nav PUBLIC mathlib filterlib clib)
