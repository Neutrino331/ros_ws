cmake_minimum_required(VERSION 3.8)
project(serial-imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(serial_imu src/serial_imu.cpp src/imu_data_decode.c src/packet.c)
add_definitions(-std=c99)
target_link_libraries(serial_imu ${catkin_LIBRARIES})

add_executable(imu_subscriber src/subscriber.cpp src/imu_data_decode.c src/packet.c)
target_link_libraries(imu_subscriber ${catkin_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
