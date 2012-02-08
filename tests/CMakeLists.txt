# Set the tests model directory.
add_definitions(
  "-DTEST_MODEL_DIRECTORY='\"${PROJECT_SOURCE_DIR}/tests/models\"'")

rosbuild_add_gtest(one_dof_revolute_x one_dof_revolute_x.cpp)
target_link_libraries(one_dof_revolute_x ${PROJECT_NAME})
