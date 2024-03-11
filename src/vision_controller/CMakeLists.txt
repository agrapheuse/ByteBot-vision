cmake_minimum_required(VERSION 3.8)
project(vision_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

# Install python scripts
install(PROGRAMS
  scripts/detect_colour_node.py
  scripts/detect_hand_gestures_im_raw_node.py
  scripts/follow_legs_node.py
  scripts/follow_person_node.py
  scripts/get_images_from_oakd_node.py
  scripts/detect_body_node.py
  scripts/change_state_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the model
install(DIRECTORY
  vision_controller/model
  DESTINATION share/${PROJECT_NAME}
)

# Install the temporary pictures
install(DIRECTORY
  vision_controller/tmp_img
  DESTINATION share/${PROJECT_NAME}
)

# Install the state json file
install(DIRECTORY
  vision_controller/state
  DESTINATION share/${PROJECT_NAME}
)

ament_package()