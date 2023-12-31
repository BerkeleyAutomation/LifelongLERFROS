cmake_minimum_required(VERSION 3.8)
project(camera_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(lifelong_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/image_visualization.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/collect_images_for_nerf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/color_image_compression_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nerf_collection_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/realsense_compressed_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/realsense_uncompression_compression.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/new_realsense_compression_uncompression.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/compress_realsense_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/image_pose_filtering.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/4_arducam_compressed_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/view_4_cam.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
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
