set(_AMENT_PACKAGE_NAME "theora_image_transport")
set(theora_image_transport_VERSION "2.5.0")
set(theora_image_transport_MAINTAINER "Kenji Brameld <kenjibrameld@gmail.com>")
set(theora_image_transport_BUILD_DEPENDS "cv_bridge" "image_transport" "libogg" "libopencv-imgproc-dev" "libtheora" "pluginlib" "rclcpp" "rcutils" "sensor_msgs" "std_msgs")
set(theora_image_transport_BUILDTOOL_DEPENDS "ament_cmake" "rosidl_default_generators" "pkg-config")
set(theora_image_transport_BUILD_EXPORT_DEPENDS "cv_bridge" "image_transport" "libogg" "libopencv-imgproc-dev" "libtheora" "pluginlib" "rclcpp" "rcutils" "sensor_msgs" "std_msgs")
set(theora_image_transport_BUILDTOOL_EXPORT_DEPENDS )
set(theora_image_transport_EXEC_DEPENDS "rosidl_default_runtime" "cv_bridge" "image_transport" "libogg" "libopencv-imgproc-dev" "libtheora" "pluginlib" "rclcpp" "rcutils" "sensor_msgs" "std_msgs")
set(theora_image_transport_TEST_DEPENDS "ament_lint_common")
set(theora_image_transport_GROUP_DEPENDS )
set(theora_image_transport_MEMBER_OF_GROUPS "rosidl_interface_packages")
set(theora_image_transport_DEPRECATED "")
set(theora_image_transport_EXPORT_TAGS)
list(APPEND theora_image_transport_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND theora_image_transport_EXPORT_TAGS "<image_transport plugin=\"theora_plugins.xml\"/>")
