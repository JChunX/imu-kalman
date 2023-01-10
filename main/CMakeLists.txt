idf_component_register(SRCS "main.cpp" "MPUReader.cpp"
                    INCLUDE_DIRS "/usr/local/include")

add_definitions(-DIIR1_NO_EXCEPTIONS)

function(install)
endfunction()

function(export)
endfunction()

add_subdirectory(lib/iir1)
target_link_libraries(${COMPONENT_LIB} PUBLIC iir)