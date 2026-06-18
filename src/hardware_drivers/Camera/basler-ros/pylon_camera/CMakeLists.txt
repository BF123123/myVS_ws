cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

add_definitions("-std=gnu++11")

set(CATKIN_COMPONENTS actionlib camera_control_msgs camera_info_manager cv_bridge diagnostic_updater image_geometry image_transport roscpp roslaunch sensor_msgs)

# ================= 【终极修复核心】 =================
execute_process(COMMAND /opt/pylon/bin/pylon-config --libs OUTPUT_VARIABLE PYLON_LIBS_RAW OUTPUT_STRIP_TRAILING_WHITESPACE)
separate_arguments(Pylon_LIBRARIES UNIX_COMMAND ${PYLON_LIBS_RAW})
set(Pylon_INCLUDE_DIRS "/opt/pylon/include")
# ==================================================

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS} roslint)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Pylon_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/binary_exposure_search.cpp src/${PROJECT_NAME}/encoding_conversions.cpp src/${PROJECT_NAME}/${PROJECT_NAME}.cpp src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp src/${PROJECT_NAME}/${PROJECT_NAME}_parameter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Pylon_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Pylon_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

add_executable(write_device_user_id_to_camera src/${PROJECT_NAME}/write_device_user_id_to_camera.cpp)
target_link_libraries(write_device_user_id_to_camera ${Pylon_LIBRARIES})

catkin_python_setup()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch FILES_MATCHING PATTERN "*.launch")
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config FILES_MATCHING PATTERN "*.yaml")
install(PROGRAMS scripts/file_sequencer.py scripts/grab_and_save_image_action_server.py scripts/result_bag_to_action.py scripts/sequence_to_file.py scripts/toggle_camera DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node write_device_user_id_to_camera LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "internal" EXCLUDE)