cmake_minimum_required(VERSION 3.10)

project(GF_BOX2D
  LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

find_package(gf REQUIRED)
find_package(Box2D REQUIRED)

add_executable(example_gf_box2d
  gfb2d/PhysicsDebugger.cc
  gfb2d/PhysicsModel.cc
  example/main.cc
)

target_compile_options(example_gf_box2d
  PRIVATE
    "-Wall" "-Wextra"
)

target_compile_features(example_gf_box2d
  PUBLIC
    cxx_std_14
)

set_target_properties(example_gf_box2d
  PROPERTIES
    CXX_EXTENSIONS OFF
)

target_include_directories(example_gf_box2d
  PRIVATE
    "${BOX2D_INCLUDE_DIRS}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(example_gf_box2d
  PRIVATE
    gf::graphics
    ${BOX2D_LIBRARIES}
)
