cmake_minimum_required(VERSION 3.12)

project(qrcode)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CMAKE_SOURCE_DIR}/src)

aux_source_directory(test TEST_SRC_LIST)
add_executable(unittest
	${TEST_SRC_LIST}
	src/ExtFiniteField256.cpp
)

target_link_libraries(unittest gtest pthread  ${OpenCV_LIBS})

target_compile_options(unittest PUBLIC -O0 -g --coverage)
#target_compile_options(unittest PUBLIC -O2)
#target_compile_options(qrcode PUBLIC -O2)

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

add_custom_target(doxygen
  COMMAND doxygen
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)



add_executable(reedsolomon
	src/main_reedsolomon.cpp
	src/ExtFiniteField256.cpp
)

add_executable(qrcode
	src/main_qrcode.cpp
	src/ExtFiniteField256.cpp
)

target_compile_options(qrcode PUBLIC -O0 -g)

target_link_libraries( qrcode ${OpenCV_LIBS} )

