cmake_minimum_required(VERSION 3.16)
project(arch_nav_supervisor_oas VERSION 0.0.1 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

if(NOT TARGET arch_nav::arch_nav)
  find_package(arch_nav REQUIRED)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/oas_config.cpp
  src/avoidance_computer.cpp
  src/threat_assessor.cpp
  src/mission_restructurer.cpp
  src/rangefinder_sensor_hub.cpp
  src/oas_supervisor.cpp
  src/oas_registration.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    arch_nav::arch_nav
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    yaml-cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/arch_nav/supervisors
)

install(FILES config/oas_default.yaml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/arch_nav/config
)

if(BUILD_TESTING)
  include(CTest)
  find_package(GTest REQUIRED)

  add_executable(test_avoidance_computer
    test/test_avoidance_computer.cpp
    src/avoidance_computer.cpp
  )
  target_link_libraries(test_avoidance_computer PRIVATE GTest::gtest_main)
  target_include_directories(test_avoidance_computer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME test_avoidance_computer COMMAND test_avoidance_computer)

  add_executable(test_threat_assessor
    test/test_threat_assessor.cpp
    src/threat_assessor.cpp
  )
  target_link_libraries(test_threat_assessor PRIVATE GTest::gtest_main)
  target_include_directories(test_threat_assessor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME test_threat_assessor COMMAND test_threat_assessor)

  add_executable(test_mission_restructurer
    test/test_mission_restructurer.cpp
    src/mission_restructurer.cpp
  )
  target_link_libraries(test_mission_restructurer PRIVATE
    GTest::gtest_main arch_nav::arch_nav)
  target_include_directories(test_mission_restructurer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME test_mission_restructurer COMMAND test_mission_restructurer)
endif()
