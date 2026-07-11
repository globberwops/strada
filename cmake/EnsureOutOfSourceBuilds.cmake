# cmake/EnsureOutOfSourceBuilds.cmake
#
# Prevents configuring the project inside the source directory.

function(strada_ensure_out_of_source_builds)
  # Make sure the user doesn't play dirty with symlinks
  file(REAL_PATH "${CMAKE_SOURCE_DIR}" srcdir)
  file(REAL_PATH "${CMAKE_BINARY_DIR}" bindir)
  if("${srcdir}" STREQUAL "${bindir}")
    message(
      FATAL_ERROR
        "In-source builds are not allowed. Please configure the project using CMake presets (e.g., cmake --preset dev)."
    )
  endif()
endfunction()

strada_ensure_out_of_source_builds()
