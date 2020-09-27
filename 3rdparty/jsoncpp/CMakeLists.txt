project(jsoncpp C)

cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

# SET LIBNAME
set(jsoncpp_sources
	json_tool.h
	json_reader.cpp
	json_valueiterator.inl
	json_value.cpp
	json_writer.cpp)

add_library(jsoncpp STATIC ${jsoncpp_sources})
set_target_properties(jsoncpp PROPERTIES FOLDER "3rdparty")
target_include_directories(jsoncpp PUBLIC
						   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)
