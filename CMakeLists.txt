cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES ./src/*.c)
file(GLOB CPP_SOURCES ./src/*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
include_directories("./include/")

# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

# Search for the RBDL include directory and library
find_package (RBDL REQUIRED)
find_package (Eigen3 REQUIRED)

#add_definitions(-D RBDL_USE_SIMPLE_MATH=FALSE )

# Add the include directory to the include paths
include_directories ( ${EIGEN3_INCLUDE_DIR} ${RBDL_INCLUDE_DIR} )

link_directories("/usr/local/lib")
include_directories ( "/usr/local/include")

add_subdirectory( ${PROJECT_SOURCE_DIR}/lib )
include_directories( ${PROJECT_SOURCE_DIR}/lib/include )

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES} wholeBodyCtrl rbdl qpOASES)

# And link the library against the executable
TARGET_LINK_LIBRARIES (${PROJECT}
	${RBDL_LIBRARY}
	)
TARGET_LINK_LIBRARIES (${PROJECT}
	libqpOASES.a
	)


# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)




