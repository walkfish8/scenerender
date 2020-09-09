# Find required packages

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

add_executable(demo_panotopano ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})
target_link_libraries(demo_panotopano scenerender)
SET_PROPERTY(TARGET demo_panotopano PROPERTY FOLDER "demos")