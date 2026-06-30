set(controller_SRC
  HammeringTaskNew.cpp
)

set(controller_HDR
  HammeringTaskNew.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DHammeringTaskNew_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_rtc_ros ${catkin_LIBRARIES})

add_subdirectory(states)
