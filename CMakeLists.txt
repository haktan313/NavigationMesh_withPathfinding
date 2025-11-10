
cmake_minimum_required(VERSION 3.15)
project(HNavmesSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB OUR_SOURCES "source/*.cpp" "source/*.h" 
"source/Core/*.cpp" "source/Core/*.h" "source/NavigationSystemThings/*.cpp" "source/NavigationSystemThings/*.h")
file(GLOB SHADER_FILES "source/shaders/*.vert" "source/shaders/*.frag")

set(IMGUI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/imgui.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/imgui_internal.h
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4/backends/imgui_impl_opengl3.cpp
)

add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/imgui-1.92.4)
target_link_libraries(imgui PUBLIC glfw)

add_library(glfw INTERFACE)
target_include_directories(glfw SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs/glfw-3.4.bin.WIN64/include)

target_link_libraries(glfw INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs/glfw-3.4.bin.WIN64/lib-vc2022/glfw3.lib)

add_library(glad STATIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/glad/src/glad.c)
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/libs/glad/include)

add_library(glm INTERFACE)
target_include_directories(glm SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs/glm-1.0.2)

add_executable(HNavmesSystem ${OUR_SOURCES} ${SHADER_FILES})

target_include_directories(HNavmesSystem PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/source)

target_link_libraries(HNavmesSystem PRIVATE glfw glad glm imgui opengl32 gdi32)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT HNavmesSystem)