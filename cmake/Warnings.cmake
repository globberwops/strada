# cmake/Warnings.cmake
#
# Compiler Warnings Module for Strada Based on the recommendations from:
# https://github.com/cpp-best-practices/cppbestpractices/blob/master/02-Use_the_Tools_Available.md#compilers

# Option to treat warnings as errors
option(STRADA_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

# MSVC Compiler Warnings
set(MSVC_WARNINGS
    /W4 # Baseline reasonable warnings
    /w14242 # 'identifier': conversion from 'type1' to 'type2', possible loss of data
    /w14254 # 'operator': conversion from 'type1:field_bits' to 'type2:field_bits', possible loss of data
    /w14263 # 'function': member function does not override any base class virtual member function
    /w14265 # 'classname': class has virtual functions, but destructor is not virtual
    /w14287 # 'operator': unsigned/negative constant mismatch
    /we4289 # nonstandard extension used: loop control variable declared in the for-loop is used outside
    /w14296 # 'operator': expression is always 'boolean_value'
    /w14311 # 'variable': pointer truncation from 'type1' to 'type2'
    /w14545 # expression before comma evaluates to a function which is missing an argument list
    /w14546 # function call before comma missing argument list
    /w14547 # 'operator': operator before comma has no effect; expected operator with side-effect
    /w14549 # 'operator': operator before comma has no effect; did you intend 'operator'?
    /w14555 # expression has no effect; expected expression with side-effect
    /w14619 # pragma warning: there is no warning number 'number'
    /w14640 # Enable warning on thread un-safe static member initialization
    /w14826 # Conversion from 'type1' to 'type2' is sign-extended.
    /w14905 # wide string literal cast to 'LPSTR'
    /w14906 # string literal cast to 'LPWSTR'
    /w14928 # illegal copy-initialization; more than one user-defined conversion has been implicitly applied
    /permissive- # Standards conformance mode
)

# Clang Compiler Warnings
set(CLANG_WARNINGS
    -Wall
    -Wextra # reasonable and standard
    -Wshadow # warn the user if a variable declaration shadows one from a parent context
    -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor
    -Wold-style-cast # warn for c-style casts
    -Wcast-align # warn for potential performance problem casts
    -Wunused # warn on anything being unused
    -Woverloaded-virtual # warn if you overload (not override) a virtual function
    -Wpedantic # warn if non-standard C++ is used
    -Wconversion # warn on type conversions that may lose data
    -Wsign-conversion # warn on sign conversions
    -Wnull-dereference # warn if a null dereference is detected
    -Wdouble-promotion # warn if float is implicit promoted to double
    -Wformat=2 # warn on security issues around functions that format output
    -Wimplicit-fallthrough # warn on statements that fallthrough without an explicit annotation
)

# GCC (GNU) Compiler Warnings
set(GCC_WARNINGS
    ${CLANG_WARNINGS}
    -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
    -Wduplicated-cond # warn if if / else chain has duplicated conditions
    -Wduplicated-branches # warn if if / else branches have duplicated code
    -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
    -Wuseless-cast # warn if you perform a cast to the same type
    -Wsuggest-override # warn if an overridden member function is not marked 'override' or 'final'
)

# Choose warning flags based on active compiler
if(MSVC)
  set(PROPOSED_WARNINGS ${MSVC_WARNINGS})
elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  set(PROPOSED_WARNINGS ${CLANG_WARNINGS})
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(PROPOSED_WARNINGS ${GCC_WARNINGS})
else()
  message(STATUS "No compiler warnings set for CXX compiler: '${CMAKE_CXX_COMPILER_ID}'")
endif()

# Treat warnings as errors if requested
if(STRADA_WARNINGS_AS_ERRORS)
  if(MSVC)
    list(APPEND PROPOSED_WARNINGS /WX)
  else()
    list(APPEND PROPOSED_WARNINGS -Werror)
  endif()
endif()

include(CheckCXXCompilerFlag)

set(SUPPORTED_WARNINGS "")
foreach(FLAG IN LISTS PROPOSED_WARNINGS)
  # Strip leading hyphen, slash, or plus sign to clean up the variable name
  string(REGEX REPLACE "^[-/+]" "" CLEAN_FLAG "${FLAG}")
  # Replace other non-alphanumeric characters with underscore
  string(REGEX REPLACE "[^a-zA-Z0-9_]" "_" FLAG_VAR "HAVE_${CLEAN_FLAG}")
  check_cxx_compiler_flag("${FLAG}" ${FLAG_VAR})
  if(${FLAG_VAR})
    list(APPEND SUPPORTED_WARNINGS "${FLAG}")
  endif()
endforeach()

# Define target helper function to apply compiler warnings
function(strada_add_warnings target)
  target_compile_options(${target} PRIVATE ${SUPPORTED_WARNINGS})
endfunction()
