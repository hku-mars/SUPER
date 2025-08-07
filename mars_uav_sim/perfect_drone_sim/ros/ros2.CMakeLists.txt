cmake_minimum_required(VERSION 3.5)
project(perfect_drone_sim)

if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif ()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# for CLION to search the package
set(CMAKE_PREFIX_PATH ../../../../install)

## ROS Related packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(marsim_render REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Other packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
        SYSTEM
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        )

find_package(marsim_render REQUIRED)

set(dependencies
        marsim_render
        rclcpp
        sensor_msgs
        mars_quadrotor_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        tf2_ros
        pcl_conversions
)

add_executable(perfect_drone_node
        src/ros2_perfect_drone_node.cpp
)

ament_target_dependencies(perfect_drone_node PUBLIC
        ${dependencies}
)
target_link_libraries(perfect_drone_node PUBLIC
        ${THIRD_PARTY}
)

target_include_directories(perfect_drone_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# 注册 导出头文件
install(
        DIRECTORY include/
        DESTINATION include
)



# 注册 导出库文件
install(
        TARGETS perfect_drone_node # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT perfect_drone_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install the launch file
install(DIRECTORY
        launch
        rviz
        rviz2
        meshes
        config
        DESTINATION share/${PROJECT_NAME}/
        )


ament_package()