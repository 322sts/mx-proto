# Pure C++ physics library: no dependency allowed here (future Unreal Engine 5 port).
add_library(mx_physics STATIC
  Math2D.h
  BikeParams.h BikeParams.cpp
  Terrain.h Terrain.cpp
  BikeSim.h BikeSim.cpp)
target_include_directories(mx_physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(MSVC)
  target_compile_options(mx_physics PRIVATE /W4)
endif()
