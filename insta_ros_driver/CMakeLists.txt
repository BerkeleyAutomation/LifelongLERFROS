cmake_minimum_required(VERSION 3.8)
project(insta_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories (include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(h264_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_AVDEVICE REQUIRED libavdevice>=58)
pkg_check_modules(PC_AVFORMAT REQUIRED libavformat>=58)
pkg_check_modules(PC_AVCODEC REQUIRED libavcodec>=58)
pkg_check_modules(PC_AVUTIL REQUIRED libavutil>=56)
pkg_check_modules(PC_SWSCALE REQUIRED libswscale>=5)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")

add_executable(insta_fisheye src/insta_fisheye_publisher.cpp)
add_executable(h264_subscriber src/h264_subscriber.cpp)
target_link_libraries(
  h264_subscriber
  ${PC_AVDEVICE_LIBRARIES}
  ${PC_AVFORMAT_LIBRARIES}
  ${PC_AVCODEC_LIBRARIES}
  ${PC_AVUTIL_LIBRARIES}
  ${PC_SWSCALE_LIBRARIES}
)
add_library(CameraSDK SHARED IMPORTED)
set_property(TARGET CameraSDK PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/libCameraSDK.so")
target_link_libraries(insta_fisheye CameraSDK)
ament_target_dependencies(insta_fisheye rclcpp std_msgs sensor_msgs h264_msgs)
ament_target_dependencies(h264_subscriber rclcpp std_msgs sensor_msgs h264_msgs)

install(TARGETS
  insta_fisheye
  h264_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
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

ament_package()
