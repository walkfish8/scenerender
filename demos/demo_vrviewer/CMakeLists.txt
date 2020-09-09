# Find required packages

# List sources files
file(GLOB LIBRARY_FILES_C "*.cpp")
file(GLOB LIBRARY_FILES_H "*.h" "*.inl")

add_executable(demo_vrviewer ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
target_link_libraries(demo_vrviewer scenerender)
SET_PROPERTY(TARGET demo_vrviewer PROPERTY FOLDER "demos")