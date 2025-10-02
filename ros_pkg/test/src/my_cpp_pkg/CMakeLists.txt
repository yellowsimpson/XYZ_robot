cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

# 먼저 ament_cmake 와 rclcpp 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 실행 파일 생성
add_executable(my_node src/my_node.cpp)

# 의존성 연결
ament_target_dependencies(my_node rclcpp)

# 실행 파일 설치 경로
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

