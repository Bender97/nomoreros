cmake_minimum_required(VERSION 3.5)
project(projecting)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED )


include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        )

add_library(Features src/Features.cpp include/Features.h)
target_link_libraries(Features ${OpenCV_LIBRARIES} )

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} Features )

install(TARGETS
        ${PROJECT_NAME} ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
        )
