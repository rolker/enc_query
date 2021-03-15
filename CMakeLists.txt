cmake_minimum_required(VERSION 2.8.3)
project(enc_query)


find_package(catkin REQUIRED COMPONENTS
  marine_msgs
  project11
  rospy
  message_generation
  std_msgs
  geographic_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  enc_feature_msg.msg
)

add_service_files(
  FILES
  enc_query_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geographic_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/enc_query_node.py nodes/enc_query_client.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
