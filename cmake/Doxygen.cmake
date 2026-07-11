# cmake/Doxygen.cmake
#
# Configures Doxygen documentation target if Doxygen is available.

function(strada_enable_doxygen)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    # Set basic Doxygen configuration options
    set(DOXYGEN_PROJECT_NAME "Strada")
    set(DOXYGEN_PROJECT_NUMBER "${PROJECT_VERSION}")
    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/docs/doxygen")
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${PROJECT_SOURCE_DIR}/README.md")
    set(DOXYGEN_EXCLUDE_PATTERNS "*/external/*" "*/build/*" "*/tests/*" "*/examples/*")
    set(DOXYGEN_GENERATE_LATEX NO)

    # Create the target
    doxygen_add_docs(strada_doxygen "${PROJECT_SOURCE_DIR}/src" "${PROJECT_SOURCE_DIR}/include"
                     "${PROJECT_SOURCE_DIR}/README.md" COMMENT "Generating API documentation with Doxygen")
    message(STATUS "Doxygen target 'strada_doxygen' configured successfully.")
  else()
    message(STATUS "Doxygen not found. Target 'strada_doxygen' will not be available.")
  endif()
endfunction()
