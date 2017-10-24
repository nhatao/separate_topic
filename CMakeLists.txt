cmake_minimum_required(VERSION 2.8.3)
project(separate_topic)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable(separate_topic src/separate_topic.cpp)
target_link_libraries(separate_topic ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
    separate_topic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
