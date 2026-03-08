# 빌드 시스템 변경사항

이 문서는 ROS1 catkin에서 ROS2 ament_cmake로의 빌드 시스템 변경사항을 설명합니다.

## 목차

- [개요](#개요)
- [빌드 도구 비교](#빌드-도구-비교)
- [CMakeLists.txt 변환](#cmakeliststxt-변환)
- [package.xml 변환](#packagexml-변환)
- [빌드 명령어](#빌드-명령어)
- [설치 규칙](#설치-규칙)
- [문제 해결](#문제-해결)

---

## 개요

### ROS1 (catkin)
- **빌드 도구**: catkin_make, catkin_tools
- **워크스페이스 구조**: `devel/`, `build/`
- **소싱**: `source devel/setup.bash`

### ROS2 (ament_cmake)
- **빌드 도구**: colcon
- **워크스페이스 구조**: `install/`, `build/`, `log/`
- **소싱**: `source install/setup.bash`

---

## 빌드 도구 비교

### catkin_make (ROS1)

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**특징:**
- 전체 워크스페이스 한번에 빌드
- CMake 래퍼
- 단일 빌드 디렉토리

### catkin build (ROS1, catkin_tools)

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

**특징:**
- 패키지별 독립 빌드
- 병렬 빌드 지원
- 격리된 빌드 환경

### colcon (ROS2)

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**특징:**
- 패키지별 독립 빌드 (catkin build와 유사)
- 다양한 빌드 시스템 지원 (ament_cmake, ament_python, cmake, setuptools)
- 병렬 빌드 기본 활성화
- 심볼릭 링크 설치 옵션

---

## CMakeLists.txt 변환

### 기본 구조

#### ROS1
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

#### ROS2
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

# C++ 표준 및 경고 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include 디렉토리는 타겟별로 설정
```

### 실행 파일 생성

#### ROS1
```cmake
add_executable(my_node src/my_node.cpp)

target_link_libraries(my_node
  ${catkin_LIBRARIES}
)
```

#### ROS2
```cmake
add_executable(my_node src/my_node.cpp)

# 헤더 파일 경로 (Generator Expression 사용)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 의존성 자동 처리
ament_target_dependencies(my_node
  rclcpp
  std_msgs
)
```

**주요 차이점:**
1. `ament_target_dependencies()`: 헤더, 라이브러리, 링크 플래그 자동 처리
2. Generator Expression: 빌드/설치 시 다른 경로 지원
3. `target_link_libraries()` 대신 `ament_target_dependencies()` 권장

### 라이브러리 생성

#### ROS1
```cmake
add_library(${PROJECT_NAME}
  src/lib1.cpp
  src/lib2.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
```

#### ROS2
```cmake
add_library(${PROJECT_NAME}
  src/lib1.cpp
  src/lib2.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
)

# 라이브러리 export (다른 패키지에서 사용 가능)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs)
```

### 외부 라이브러리 사용 (Boost, OpenCV 등)

#### ROS1
```cmake
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(my_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

#### ROS2
```cmake
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)

add_executable(my_node src/my_node.cpp)

target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(my_node
  rclcpp
  std_msgs
)

target_link_libraries(my_node
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
```

**주요 차이점:**
- ROS 패키지 의존성: `ament_target_dependencies()`
- 외부 라이브러리: `target_link_libraries()` (동일)

---

## package.xml 변환

### 기본 구조

#### ROS1 (format 2)
```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_package</name>
  <version>0.1.0</version>
  <description>My package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>

  <build_depend>boost</build_depend>
  <exec_depend>some_tool</exec_depend>
</package>
```

#### ROS2 (format 3)
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.1.0</version>
  <description>My package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <build_depend>boost</build_depend>
  <exec_depend>some_tool</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**주요 차이점:**
1. `format="3"` 필수
2. `catkin` → `ament_cmake`
3. `roscpp` → `rclcpp`
4. `<export>` 섹션 추가

### 의존성 태그

| 태그 | 용도 |
|------|------|
| `<buildtool_depend>` | 빌드 도구 (ament_cmake) |
| `<build_depend>` | 빌드 시에만 필요 |
| `<exec_depend>` | 실행 시에만 필요 |
| `<depend>` | 빌드 + 실행 모두 필요 |
| `<test_depend>` | 테스트 시에만 필요 |

### 실제 예시: arduino_driver

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arduino_driver</name>
  <version>0.1.0</version>
  <description>Arduino serial communication driver</description>

  <maintainer email="deepblue@example.com">DeepBlue</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- ROS2 의존성 -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>ackermann_msgs</depend>

  <!-- 외부 라이브러리 (빌드 시에만) -->
  <build_depend>boost</build_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 빌드 명령어

### 전체 빌드

```bash
# ROS1
cd ~/catkin_ws
catkin_make
# 또는
catkin build

# ROS2
cd ~/ros2_ws
colcon build
```

### 특정 패키지만 빌드

```bash
# ROS1
catkin build my_package

# ROS2
colcon build --packages-select my_package
```

### 특정 패키지 및 의존성 빌드

```bash
# ROS1
catkin build my_package --no-deps  # 의존성 제외
catkin build my_package             # 의존성 포함

# ROS2
colcon build --packages-select my_package  # 의존성 제외
colcon build --packages-up-to my_package   # 의존성 포함
```

### 병렬 빌드

```bash
# ROS1
catkin build -j4  # 4개 작업 병렬

# ROS2
colcon build --parallel-workers 4
```

### 심볼릭 링크 설치 (개발 모드)

```bash
# ROS1
catkin build --no-install  # devel 공간 사용

# ROS2
colcon build --symlink-install  # Python 파일 수정 시 재빌드 불필요
```

### 클린 빌드

```bash
# ROS1
catkin clean
# 또는
rm -rf build devel

# ROS2
rm -rf build install log
colcon build
```

---

## 설치 규칙

### 실행 파일 설치

#### ROS1
```cmake
install(TARGETS my_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### ROS2
```cmake
install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)
```

### 라이브러리 설치

#### ROS1
```cmake
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
```

#### ROS2
```cmake
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)
```

### 헤더 파일 설치

#### ROS1
```cmake
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

#### ROS2
```cmake
install(DIRECTORY include/
  DESTINATION include
)
```

### Python 스크립트 설치

#### ROS1
```cmake
catkin_install_python(PROGRAMS
  scripts/my_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### ROS2
```cmake
install(PROGRAMS
  scripts/my_script.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### 런치 파일 설치

#### ROS1
```cmake
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
```

#### ROS2
```cmake
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

---

## 워크스페이스 구조

### ROS1
```
catkin_ws/
├── src/
│   ├── package1/
│   └── package2/
├── build/          # 빌드 산출물
├── devel/          # 개발용 설치
│   ├── setup.bash
│   ├── lib/
│   └── include/
└── install/        # (선택적) 릴리스 설치
```

### ROS2
```
ros2_ws/
├── src/
│   ├── package1/
│   └── package2/
├── build/          # 빌드 산출물 (패키지별 디렉토리)
│   ├── package1/
│   └── package2/
├── install/        # 설치 디렉토리 (기본)
│   ├── setup.bash
│   ├── package1/
│   └── package2/
└── log/            # 빌드 로그
    ├── latest_build/
    └── build_2026-01-29_10-30-45/
```

---

## 패키지 종류별 빌드 설정

### C++ 패키지 (ament_cmake)

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### Python 패키지 (ament_python)

**setup.py:**
```python
from setuptools import setup

package_name = 'my_python_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'my_node = my_python_package.my_node:main'
        ],
    },
)
```

**package.xml:**
```xml
<buildtool_depend>ament_python</buildtool_depend>
<exec_depend>rclpy</exec_depend>
```

---

## colcon 유용한 옵션

### 빌드 옵션

```bash
# 특정 패키지만 빌드
colcon build --packages-select pkg1 pkg2

# 특정 패키지 제외
colcon build --packages-skip pkg1

# 특정 패키지까지 (의존성 포함)
colcon build --packages-up-to pkg1

# 심볼릭 링크 설치 (Python 개발 시 유용)
colcon build --symlink-install

# 병렬 작업 수 지정
colcon build --parallel-workers 4

# CMake 인자 전달
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 테스트 옵션

```bash
# 테스트 실행
colcon test

# 특정 패키지만 테스트
colcon test --packages-select pkg1

# 테스트 결과 확인
colcon test-result --all
colcon test-result --verbose
```

### 로그 확인

```bash
# 빌드 로그 보기
cat log/latest_build/events.log

# 특정 패키지 로그
cat log/latest_build/pkg1/stdout.log
cat log/latest_build/pkg1/stderr.log
```

---

## 문제 해결

### 1. 패키지를 찾을 수 없음

**에러:**
```
CMake Error: Could not find a package configuration file provided by "package_name"
```

**해결:**
```bash
# ROS2 패키지 설치 확인
ros2 pkg list | grep package_name

# 설치 안되어 있으면
sudo apt install ros-humble-package-name
```

### 2. 헤더 파일을 찾을 수 없음

**에러:**
```
fatal error: package/header.hpp: No such file or directory
```

**해결:**
```cmake
# CMakeLists.txt에 include 경로 추가
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```

### 3. 이전 빌드 캐시 문제

**증상:**
- 수정한 코드가 반영 안됨
- 이상한 링크 에러

**해결:**
```bash
# 클린 빌드
rm -rf build install log
colcon build
```

### 4. ament_package() 누락

**에러:**
```
CMake Error: ament_package() not found
```

**해결:**
```cmake
# CMakeLists.txt 마지막에 추가
ament_package()
```

### 5. Python 스크립트 실행 권한

**에러:**
```
Permission denied: 'my_script.py'
```

**해결:**
```bash
chmod +x scripts/my_script.py
```

또는 CMakeLists.txt에서:
```cmake
install(PROGRAMS  # PROGRAMS는 자동으로 실행 권한 부여
  scripts/my_script.py
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## 요약

### ROS1 → ROS2 빌드 시스템 변경사항

| 항목 | ROS1 | ROS2 |
|------|------|------|
| 빌드 도구 | catkin_make, catkin build | colcon |
| 빌드 시스템 | catkin | ament_cmake |
| package.xml format | 2 | 3 |
| 워크스페이스 디렉토리 | devel/ | install/ |
| 의존성 처리 | target_link_libraries | ament_target_dependencies |
| 소싱 | source devel/setup.bash | source install/setup.bash |

### 체크리스트

빌드 시스템 마이그레이션 시 확인 사항:

- [ ] CMakeLists.txt 최소 버전: 3.8 이상
- [ ] `find_package(ament_cmake REQUIRED)` 추가
- [ ] `catkin_package()` → 제거
- [ ] `ament_target_dependencies()` 사용
- [ ] `ament_package()` 추가 (마지막 줄)
- [ ] `install()` 규칙 업데이트
- [ ] package.xml format="3" 설정
- [ ] `<export><build_type>` 추가
- [ ] 빌드 및 테스트 확인

---

## 참고 자료

- [ament_cmake Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [colcon User Documentation](https://colcon.readthedocs.io/)
- [ROS2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)
