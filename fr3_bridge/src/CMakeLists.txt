set(ALL_SOURCE_TARGETS)

# macro for building the executables
macro(create_executable ROBOT_NAME)
  set(EXECUTABLE_NAME ${ROBOT_NAME})

  # Robot executables.
  add_executable(${EXECUTABLE_NAME} ${ROBOT_NAME}/${ROBOT_NAME}.cpp franka_trajectory_utils.cpp)
  # Add the include dependencies.
  target_include_directories(${EXECUTABLE_NAME} PUBLIC 
                             $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                             ${fr3py_INCLUDE_DIRS} 
                             $<INSTALL_INTERFACE:include>)
  
  # Link the dependencies.
  target_link_libraries(${EXECUTABLE_NAME} 
  Franka::Franka Threads::Threads
  lcm
  yaml-cpp
  Eigen3::Eigen
  )

  list(APPEND ALL_SOURCE_TARGETS ${EXECUTABLE_NAME})
endmacro(create_executable ROBOT_NAME)

# Add excecutables
create_executable(fr3_joint_interface)
# create_executable(fr3_task_interface)

# Install targets
install(TARGETS ${ALL_SOURCE_TARGETS}
        RUNTIME DESTINATION bin)

