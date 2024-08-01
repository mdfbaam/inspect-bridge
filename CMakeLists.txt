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

    ament_target_dependencies(${MKNODE_NAME} rclcpp rclcpp_action fmt)

    target_compile_features(${MKNODE_NAME} PUBLIC c_std_99 cxx_std_17)
    target_compile_options(${MKNODE_NAME} PRIVATE -DFMT_HEADER_ONLY)
    target_link_libraries(${MKNODE_NAME} "${CPP_TYPESUPPORT_TARGET}")

    install(TARGETS ${MKNODE_NAME} DESTINATION lib/${PROJECT_NAME})
endfunction()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS Dependencies
find_package(ament_cmake               REQUIRED)
find_package(rclcpp                    REQUIRED)
find_package(rclcpp_action             REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# External Dependencies
find_package(fmt REQUIRED)

# Generate the ROS interfaces
rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "interfaces/act/Scan.action"
    "interfaces/act/Polygonize.action"
    "interfaces/act/New.action"
    "interfaces/act/Write.action"
    DEPENDENCIES action_msgs std_msgs
)

rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Actually create the nodes now.
CreateNode(
    NAME action
    SOURCES src/act/node.cpp
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
