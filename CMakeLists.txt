cmake_minimum_required(VERSION 3.12)

project(qrcode)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(qrcode
	main.cpp
	ExtFiniteField256.cpp
)

aux_source_directory(test TEST_SRC_LIST)
add_executable(unittest
	${TEST_SRC_LIST}
	ExtFiniteField256.cpp
)

target_link_libraries(unittest gtest pthread)

target_compile_options(unittest PUBLIC -O0 -g --coverage)
target_compile_options(qrcode PUBLIC -O2)
target_link_options(unittest PUBLIC --coverage)

add_custom_target(lcov
	COMMAND lcov --no-external --directory ${CMAKE_BINARY_DIR} --directory ${CMAKE_SOURCE_DIR} --capture --output-file capture.info
	COMMAND genhtml capture.info -o cov_html
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)

add_custom_target(applyCodeStyle
	COMMAND find -iname '*.h' -o -iname '*.cpp' -o -iname '*.c' | xargs clang-format -i
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
