add_executable(cloud_node cloud_node.cc convert.cc pointcloudXYZIR.cc  pointcloudXYZI.cc organized_cloudXYZIR.cc)
add_dependencies(cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_node velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(cloud_nodelet cloud_nodelet.cc convert.cc pointcloudXYZIR.cc  pointcloudXYZI.cc organized_cloudXYZIR.cc)
add_dependencies(cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cloud_nodelet velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(transform_node transform_node.cc transform.cc pointcloudXYZIR.cc  pointcloudXYZI.cc organized_cloudXYZIR.cc)
add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(transform_nodelet transform_nodelet.cc transform.cc pointcloudXYZIR.cc  pointcloudXYZI.cc organized_cloudXYZIR.cc)
add_dependencies(transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#add record
add_executable(recordpoints_node recordpoints_node.cc record.cc)
add_dependencies(recordpoints_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(recordpoints_node velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS recordpoints_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add convert_imu
add_executable(convert_imu_node convert_imu_node.cc convert_imu.cc)
add_dependencies(convert_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(convert_imu_node
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS convert_imu_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
