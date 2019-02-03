cmake_minimum_required(VERSION 3.0.2)
project(swagboy)

set(CMAKE_C_STANDARD 99)

include_directories(/home/bilal/ev3dev-c/source/ev3)
#include_directories(/home/bilal/Downloads/libastar-0.9-2/src)

add_executable(swagboy src/main.c src/wheels.c src/wheels.h src/claw.c src/claw.h src/sensors.c src/sensors.h src/controller.c src/controller.h src/pathfinding.c src/pathfinding.h src/memwatch.h)