cmake_minimum_required(VERSION 2.4)

project(Inverse_Kinematics)

add_executable(app 
		src/Inverse_Kinematics.cpp
		src/main.cpp)


find_package(catkin REQUIRED COMPONENTS
  
  std_msgs
  sensor_msgs
  geometry_msgs

)

install(FILES Inverse_Kinematics_h.h DESTINATION include)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS
  sensor_msgs
  std_msgs
  geometry_msgs
    
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}  
)
