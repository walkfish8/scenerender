# Find required packages

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# Place Common.cpp as the first file in the list
# needed by cotire when setting PCH manually
# LIST(REMOVE_ITEM LIBRARY_FILES_C ${PCH_C})
SET(LIBRARY_FILES_C "${PCH_C};${LIBRARY_FILES_C}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
SET(CAMKE_CXX_FLAGS_RELEASE "${CAMKE_CXX_FLAGS_RELEASE} -O2 -Wall")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -Wall")

add_definitions(-DSCENERENDER_EXPORTS)
add_library(scenerender SHARED ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
target_link_libraries(scenerender ${OpenCV_LIBS} ${VLD_LIBS})
SET_PROPERTY(TARGET scenerender PROPERTY FOLDER "libs")
