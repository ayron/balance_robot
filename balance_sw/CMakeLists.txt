cmake_minimum_required(VERSION 2.8.3)
project(balance_sw)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  balance_msgs
)

catkin_package(
  CATKIN_DEPENDS balance_msgs roscpp
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lwaxi src/lwaxi.cpp)
target_link_libraries(lwaxi ${catkin_LIBRARIES})
add_dependencies(lwaxi ${catkin_EXPORTED_TARGETS})

add_executable(adxl345 src/adxl345.cpp)
target_link_libraries(adxl345 ${catkin_LIBRARIES})
add_dependencies(adxl345 ${catkin_EXPORTED_TARGETS})

add_executable(currents src/currents.cpp)
target_link_libraries(currents ${catkin_LIBRARIES})
add_dependencies(currents ${catkin_EXPORTED_TARGETS})

add_executable(positions src/positions.cpp)
target_link_libraries(positions ${catkin_LIBRARIES})
add_dependencies(positions ${catkin_EXPORTED_TARGETS})

add_executable(pwms src/pwms.cpp)
target_link_libraries(pwms ${catkin_LIBRARIES})
add_dependencies(pwms ${catkin_EXPORTED_TARGETS})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller ${catkin_EXPORTED_TARGETS})

install(TARGETS lwaxi adxl345 positions currents pwms controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES launch/simple.launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

