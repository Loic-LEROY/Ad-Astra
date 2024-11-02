cmake_minimum_required(VERSION 3.16)

project(ad-astra VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
)

qt6_standard_project_setup()

file(GLOB_RECURSE SOURCES src/*.cpp)
file(GLOB_RECURSE HEADERS include/*.hpp)

include_directories(include)

qt6_add_executable(
    ad-astra
    ${SOURCES}
    ${HEADERS}
    resources/resources.qrc
)

target_link_libraries(ad-astra PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

set_target_properties(ad-astra PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
)