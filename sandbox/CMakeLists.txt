# Pinned third-party versions (downloaded once into build/_deps).
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz)
FetchContent_Declare(imgui  URL https://github.com/ocornut/imgui/archive/refs/tags/v1.93.0.tar.gz)
FetchContent_Declare(implot URL https://github.com/epezent/implot/archive/refs/tags/v1.1.tar.gz)
FetchContent_Declare(rlimgui URL https://github.com/raylib-extras/rlImGui/archive/5c17e70e98ae9f50ca94bdcad0e3b3536b46a203.tar.gz)
FetchContent_MakeAvailable(raylib imgui implot rlimgui)

add_library(mx_ui_deps STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
  ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
target_include_directories(mx_ui_deps PUBLIC ${imgui_SOURCE_DIR} ${implot_SOURCE_DIR} ${rlimgui_SOURCE_DIR})
target_link_libraries(mx_ui_deps PUBLIC raylib)

add_executable(mx_sandbox main.cpp Render.cpp Render.h)
target_link_libraries(mx_sandbox PRIVATE mx_physics mx_ui_deps)
# Run from the repo root so presets/ is found whatever the launch method.
set_target_properties(mx_sandbox PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
