cmake_minimum_required(VERSION 3.13)
project(Stag)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "Forcing C++17 standard in all targets")
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON CACHE BOOL "Forcing C11 standard in all targets")

if(MSVC)
  add_compile_options(/WX)
  add_compile_options(
    "$<$<CONFIG:RELEASE>:/MD>"
    "$<$<CONFIG:DEBUG>:/MDd>"
  )
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -w  ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -w ")
endif()

find_package(OpenCV REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
file(GLOB SRC_FILE1 "src/*.c*")
file(GLOB SRC_FILE2 "src/ED/*.c*")

add_library(
    stag_markers SHARED
    ${SRC_FILE1}
    ${SRC_FILE2}
)

target_include_directories(stag_markers
    PRIVATE src/
    PRIVATE src/ED/
    PUBLIC include/
    PUBLIC ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(stag_markers
    PRIVATE ${OpenCV_LIBS}
)

if(MSVC)
  target_compile_options(stag_markers PRIVATE /wd4018 /wd4244 /wd4267)
endif()
