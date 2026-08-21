cmake_minimum_required(VERSION 3.16)
project(mcu_protocol)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 这一层做的是字节级打包和有符号/无符号混算，-Wconversion 类告警值得当错看。
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mcu_protocol_msgs REQUIRED)

# --- 协议层：不含任何 ROS 头文件，故不链接 rclcpp。
#     这条边界靠 CMake 强制：想在这一层用 rclcpp 会直接编译失败。
add_library(mcu_protocol_core STATIC
  src/protocol.cpp
  src/parser.cpp
  src/serial_port.cpp
  src/link_state.cpp
)
target_include_directories(mcu_protocol_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(mcu_protocol_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- 桥接节点：唯一依赖 rclcpp 的部分。
add_library(mcu_bridge_component SHARED src/bridge_node.cpp)
target_include_directories(mcu_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mcu_bridge_component
  mcu_protocol_core
  rclcpp::rclcpp
  ${mcu_protocol_msgs_TARGETS}
)

add_executable(mcu_bridge src/bridge_main.cpp)
target_link_libraries(mcu_bridge mcu_bridge_component rclcpp::rclcpp)

install(TARGETS mcu_protocol_core mcu_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mcu_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- 调试用监视节点（Python）。只 exec_depend rclpy，不进构建依赖 —— 驱动本身
#     不含 Python。去掉 .py 后缀让命令读起来和 C++ 节点一致。
#     文件名刻意不带 .py：install(PROGRAMS ... RENAME) 在 --symlink-install 下退化成
#     **拷贝**，改了源文件却跑着旧版本，而且没有任何提示。不重命名才拿得到符号链接。
install(PROGRAMS scripts/mcu_monitor DESTINATION lib/${PROJECT_NAME})

# --- 测试。协议层与状态机全部在主机上跑，不需要串口也不需要硬件。
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol mcu_protocol_core)

  ament_add_gtest(test_link_state test/test_link_state.cpp)
  target_link_libraries(test_link_state mcu_protocol_core)

  # 刻意不启用 ament_lint_auto 的 copyright/uncrustify：本项目注释与文档用中文，
  # 且代码风格已统一，让 linter 在这两项上反复报错只会让人忽略测试结果。
endif()

ament_export_include_directories(include)
ament_export_libraries(mcu_protocol_core)
ament_export_dependencies(rclcpp mcu_protocol_msgs)
ament_package()
