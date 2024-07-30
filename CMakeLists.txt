cmake_minimum_required(VERSION 3.8)
project(inspect_bridge)

function(CreateNode)
    set(one_value_args NAME)
    set(multi_value_args SOURCES)
    cmake_parse_arguments(MKNODE "" "${one_value_args}" "${multi_value_args}" ${ARGN})

    add_executable(${MKNODE_NAME} ${MKNODE_SOURCES})
    target_include_directories(
        ${MKNODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_target_dependencies(${MKNODE_NAME} rclcpp fmt)

    target_compile_features(${MKNODE_NAME} PUBLIC c_std_99 cxx_std_17)
    target_compile_options(${MKNODE_NAME} PRIVATE -DFMT_HEADER_ONLY)

    install(TARGETS ${MKNODE_NAME} DESTINATION lib/${PROJECT_NAME})
endfunction()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

CreateNode(
    NAME scan
    SOURCES src/act/scan.cpp
)

CreateNode(
    NAME polygonize
    SOURCES src/act/polygonize.cpp
)

CreateNode(
    NAME bridge
    SOURCES src/service/bridge.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
