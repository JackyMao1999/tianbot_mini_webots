cmake_minimum_required(VERSION 3.0.2)
project(tianbot_mini_webots)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# 调用OpenCV配置:C++版本为11
set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf
)
find_package(OpenCV 4.2 REQUIRED) # 找OpenCV 4.2 安装路径



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoolStamped.msg
  Float64Stamped.msg
  Int32Stamped.msg
  Int8Stamped.msg
  RadarTarget.msg
  RecognitionObject.msg
  StringStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  camera_get_focus_info.srv
  camera_get_info.srv
  camera_get_zoom_info.srv
  display_draw_line.srv
  display_draw_oval.srv
  display_draw_pixel.srv
  display_draw_polygon.srv
  display_draw_rectangle.srv
  display_draw_text.srv
  display_get_info.srv
  display_image_copy.srv
  display_image_delete.srv
  display_image_load.srv
  display_image_new.srv
  display_image_paste.srv
  display_image_save.srv
  display_set_font.srv
  field_get_bool.srv
  field_get_color.srv
  field_get_count.srv
  field_get_float.srv
  field_get_int32.srv
  field_get_node.srv
  field_get_rotation.srv
  field_get_string.srv
  field_get_type.srv
  field_get_type_name.srv
  field_get_vec2f.srv
  field_get_vec3f.srv
  field_import_node.srv
  field_import_node_from_string.srv
  field_remove_node.srv
  field_remove.srv
  field_set_bool.srv
  field_set_color.srv
  field_set_float.srv
  field_set_int32.srv
  field_set_rotation.srv
  field_set_string.srv
  field_set_vec2f.srv
  field_set_vec3f.srv
  get_bool.srv
  get_float_array.srv
  get_float.srv
  get_int.srv
  get_string.srv
  get_uint64.srv
  get_urdf.srv
  gps_decimal_degrees_to_degrees_minutes_seconds.srv
  lidar_get_frequency_info.srv
  lidar_get_info.srv
  lidar_get_layer_point_cloud.srv
  lidar_get_layer_range_image.srv
  motor_set_control_pid.srv
  mouse_get_state.srv
  node_add_force_or_torque.srv
  node_add_force_with_offset.srv
  node_get_center_of_mass.srv
  node_get_contact_point.srv
  node_get_field.srv
  node_get_id.srv
  node_get_number_of_contact_points.srv
  node_get_name.srv
  node_get_orientation.srv
  node_get_parent_node.srv
  node_get_position.srv
  node_get_static_balance.srv
  node_get_status.srv
  node_get_type.srv
  node_get_velocity.srv
  node_remove.srv
  node_reset_functions.srv
  node_move_viewpoint.srv
  node_set_visibility.srv
  node_set_velocity.srv
  pen_set_ink_color.srv
  range_finder_get_info.srv
  receiver_get_emitter_direction.srv
  robot_get_device_list.srv
  robot_set_mode.srv
  robot_wait_for_user_input_event.srv
  save_image.srv
  set_bool.srv
  set_float.srv
  set_float_array.srv
  set_int.srv
  set_string.srv
  skin_get_bone_name.srv
  skin_get_bone_orientation.srv
  skin_get_bone_position.srv
  skin_set_bone_orientation.srv
  skin_set_bone_position.srv
  speaker_is_sound_playing.srv
  speaker_speak.srv
  speaker_play_sound.srv
  supervisor_get_from_def.srv
  supervisor_get_from_id.srv
  supervisor_movie_start_recording.srv
  supervisor_set_label.srv
  supervisor_virtual_reality_headset_get_orientation.srv
  supervisor_virtual_reality_headset_get_position.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tianbot_volcano
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs sensor_msgs message_runtime tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # 加载OpenCV 4.2 安装路径
)

add_executable(tianbot_velocity src/tianbot_velocity.cpp)
add_dependencies(tianbot_velocity webots_ros_generate_messages_cpp)
target_link_libraries(tianbot_velocity	${catkin_LIBRARIES})

add_executable(tianbot_gmapping_bringup src/tianbot_gmapping_bringup.cpp)
add_dependencies(tianbot_gmapping_bringup webots_ros_generate_messages_cpp)
target_link_libraries(tianbot_gmapping_bringup	${catkin_LIBRARIES})

add_executable(tianbot_2dnav_move src/tianbot_2dnav_move.cpp)
add_dependencies(tianbot_2dnav_move webots_ros_generate_messages_cpp)
target_link_libraries(tianbot_2dnav_move	${catkin_LIBRARIES})

add_executable(tianbot_line_follow src/tianbot_line_follow.cpp)
add_dependencies(tianbot_line_follow webots_ros_generate_messages_cpp)
target_link_libraries(tianbot_line_follow	${catkin_LIBRARIES})
target_link_libraries(tianbot_line_follow	${OpenCV_LIBS})