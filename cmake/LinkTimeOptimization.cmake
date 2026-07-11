# cmake/LinkTimeOptimization.cmake
#
# Enables Link-Time Optimization (LTO/IPO) on a target if compiler-supported.

function(strada_enable_lto target)
  if(STRADA_ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT result OUTPUT output)
    if(result)
      set_target_properties(${target} PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE ON
                                                 INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
      message(STATUS "Link-Time Optimization (IPO/LTO) enabled for target: ${target}")
    else()
      message(STATUS "Link-Time Optimization (IPO/LTO) is not supported by compiler for target ${target}: ${output}")
    endif()
  endif()
endfunction()
