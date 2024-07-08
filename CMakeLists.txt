##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.5.0)
project(robot_operator)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mobile_base_msgs
  cv_bridge
  rviz
  audio_common_msgs
)

find_package(ALSA REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Qwt REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}
  include/robot_operator
)

catkin_package()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robot_operator/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(robot_operator ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

target_link_libraries(robot_operator Qt5::Widgets Qt5::Gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ALSA_LIBRARIES} Qwt::Qwt)

install(TARGETS robot_operator RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
