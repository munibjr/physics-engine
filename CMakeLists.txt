cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Main library
add_library(physics_engine
    src/vector2.cpp
    src/rigidbody.cpp
    src/collision.cpp
    src/world.cpp
    src/broadphase.cpp
)

target_include_directories(physics_engine PUBLIC include)

# Examples
add_executable(ball_drop examples/ball_drop.cpp)
target_link_libraries(ball_drop physics_engine)

add_executable(stacking examples/stacking.cpp)
target_link_libraries(stacking physics_engine)

add_executable(constraints examples/constraints.cpp)
target_link_libraries(constraints physics_engine)

# Tests
enable_testing()
add_executable(test_vector2 tests/test_vector2.cpp)
target_link_libraries(test_vector2 physics_engine)
add_test(NAME Vector2Tests COMMAND test_vector2)

add_executable(test_collision tests/test_collision.cpp)
target_link_libraries(test_collision physics_engine)
add_test(NAME CollisionTests COMMAND test_collision)

add_executable(test_world tests/test_world.cpp)
target_link_libraries(test_world physics_engine)
add_test(NAME WorldTests COMMAND test_world)

# Benchmarks
add_executable(benchmark_physics benchmarks/benchmark.cpp)
target_link_libraries(benchmark_physics physics_engine)
