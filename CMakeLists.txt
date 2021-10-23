cmake_minimum_required(VERSION 3.12)

project(qrcode)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -fprofile-arcs -ftest-coverage")

include_directories(${CMAKE_SOURCE_DIR})

add_executable(${PROJECT_NAME}
	main.cpp
)

aux_source_directory(test TEST_SRC_LIST)
add_executable(unittest ${TEST_SRC_LIST})
target_link_libraries(unittest gtest pthread)

add_custom_target(lcov
	COMMAND lcov --no-external --directory ${CMAKE_BINARY_DIR} --directory ${CMAKE_SOURCE_DIR} --capture --output-file capture.info
	COMMAND genhtml capture.info -o cov_html
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)

add_custom_target(applyCodeStyle
	COMMAND find -iname '*.h' -o -iname '*.cpp' -o -iname '*.c' | xargs clang-format -i
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
