cmake_minimum_required(VERSION 3.8)
project(custom_mavlink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找构建依赖
find_package(ament_cmake REQUIRED)

# =======================================================
# 💡 核心新增代码：安装头文件并向外导出
# =======================================================
# 把 include 文件夹下的所有东西安装到系统的 include 目录
install(DIRECTORY include/
  DESTINATION include
)

# 告诉 ROS 2 系统：如果有别的包依赖我，请把我的 include 目录提供给它们
ament_export_include_directories(include)
# =======================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()