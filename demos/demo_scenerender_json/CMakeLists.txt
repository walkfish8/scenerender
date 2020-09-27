# Find required packages

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

add_executable(demo_scenerender_json ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
target_link_libraries(demo_scenerender_json scenerender jsoncpp)
SET_PROPERTY(TARGET demo_scenerender_json PROPERTY FOLDER "demos")