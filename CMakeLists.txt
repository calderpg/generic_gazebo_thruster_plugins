cmake_minimum_required(VERSION 2.8.3)
project(generic_gazebo_thruster_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

find_package(Boost REQUIRED COMPONENTS system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

if (GAZEBO_FOUND)
  message(STATUS "Found Gazebo with ${GAZEBO_INCLUDE_DIRS} (include path)")
else()
  message(FATAL_ERROR "Gazebo not found")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project
catkin_package(LIBRARIES
               generic_gazebo_force_thruster_plugin
               generic_gazebo_torque_thruster_plugin
               generic_gazebo_wrench_thruster_plugin
               CATKIN_DEPENDS
               roscpp
               std_msgs
               geometry_msgs)

###########
## Build ##
###########

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS}
                           ${GAZEBO_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})

## Build options
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)
add_compile_options(-Werror=non-virtual-dtor)

add_library(generic_gazebo_force_thruster_plugin
            src/${PROJECT_NAME}/generic_gazebo_force_thruster_plugin.cpp)
target_link_libraries(generic_gazebo_force_thruster_plugin
                      ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
                      ${Boost_LIBRARIES})
add_dependencies(generic_gazebo_force_thruster_plugin
                 ${catkin_EXPORTED_TARGETS})

add_library(generic_gazebo_torque_thruster_plugin
            src/${PROJECT_NAME}/generic_gazebo_torque_thruster_plugin.cpp)
target_link_libraries(generic_gazebo_torque_thruster_plugin
                      ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
                      ${Boost_LIBRARIES})
add_dependencies(generic_gazebo_torque_thruster_plugin
                 ${catkin_EXPORTED_TARGETS})

add_library(generic_gazebo_wrench_thruster_plugin
            src/${PROJECT_NAME}/generic_gazebo_wrench_thruster_plugin.cpp)
target_link_libraries(generic_gazebo_wrench_thruster_plugin
                      ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
                      ${Boost_LIBRARIES})
add_dependencies(generic_gazebo_wrench_thruster_plugin
                 ${catkin_EXPORTED_TARGETS})

## Mark library for installation
install(TARGETS generic_gazebo_force_thruster_plugin
                generic_gazebo_torque_thruster_plugin
                generic_gazebo_wrench_thruster_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
