cmake_minimum_required(VERSION 3.0.2)
project(amr_docker)

add_compile_options(-std=c++17)
set (CMAKE_SHARED_LINKER_FLAGS "-static-libstdc++")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  dynamic_reconfigure
)

include_directories(
  include(${catkin_INCLUDE_DIRS})
)

# dynamic reconfig
# generate_dynamic_reconfigure_options(
#   cfg/docker.cfg
#   )

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS roscpp 
  DEPENDS
)


set(SOURCES1 node/amr_docker_node.cpp src/amr_docker.cpp)
add_executable(amr_docker_node ${SOURCES1})
# add_dependencies(amr_docker_node ${PROJECT_NAME}_gencfg)
target_link_libraries(amr_docker_node
${roscpp_LIBRARIES}
${catkin_LIBRARIES}
)

# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})
# enable_testing()
# add_executable(
#     utest
#     test/utest.cpp
#   )
#   target_link_libraries(
#   utest
#   ${GTEST_LIBRARIES}
#   pthread
# )

# gtest_discover_tests(utest)

install(TARGETS amr_docker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
