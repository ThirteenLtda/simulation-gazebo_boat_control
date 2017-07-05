rock_library(gazebo_boat_controller
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(gazebo_boat_controller_bin Main.cpp
    DEPS gazebo_boat_controller)

