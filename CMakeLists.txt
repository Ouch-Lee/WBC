cmake_minimum_required( VERSION 3.0 )

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2 -Wall")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package( PythonLibs 3 REQUIRED )
# include_directories( ${PYTHON_INCLUDE_DIRS} )

# Search for the Eigen3 include directory and library
find_package (Eigen3 REQUIRED)
include_directories ( ${EIGEN3_INCLUDE_DIR} )

#link_directories("/usr/local/lib")
#include_directories("/usr/local/include")

add_subdirectory( ${PROJECT_SOURCE_DIR}/lib )
include_directories( ${PROJECT_SOURCE_DIR}/lib/include )

aux_source_directory( ${PROJECT_SOURCE_DIR}/src SRC_DIR )
include_directories( ${PROJECT_SOURCE_DIR}/include )

# Link the Webots controller library.
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
    )

# Add the RBDL library
add_library(rbdl STATIC IMPORTED)
set_property(TARGET rbdl PROPERTY IMPORTED_LOCATION /usr/local/lib/librbdl.a)

# Add the qpOASES library
add_library(qpOASES STATIC IMPORTED)
set_property(TARGET qpOASES PROPERTY IMPORTED_LOCATION /usr/local/lib/libqpOASES.a)

# Setup the target executable.
add_executable(${PROJECT} ${SRC_DIR})
target_link_libraries(${PROJECT} ${LIBRARIES} wholeBodyCtrl rbdl qpOASES)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR})



