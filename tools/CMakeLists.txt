# tools/ — host-side developer tools. Nothing here is a dependency of
# branes::reflex.

# reflex-trace/1 writer (header-only), for scenario executables that produce
# the traces the docs-site player renders.
add_library(reflex_trace INTERFACE)
add_library(branes::reflex_trace ALIAS reflex_trace)
target_include_directories(reflex_trace INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/trace/include>)
target_compile_features(reflex_trace INTERFACE cxx_std_20)

# One executable per scenario source, named trace_<block>_<scenario> (e.g.
# trace_quadrotor_roll_pulse). Each writes $REFLEX_TRACE_DIR/<block>/<scenario>.json;
# scripts/gen-traces.sh runs them all. The reflex_traces target builds them.
file(GLOB_RECURSE TRACE_SCENARIOS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/trace/scenarios/*.cpp)
add_custom_target(reflex_traces)
foreach(source ${TRACE_SCENARIOS})
    get_filename_component(stem ${source} NAME_WE)
    get_filename_component(dir ${source} DIRECTORY)
    get_filename_component(block ${dir} NAME)
    set(name trace_${block}_${stem})
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE branes::reflex branes::reflex_trace)
    reflex_set_warnings(${name})
    set_target_properties(${name} PROPERTIES
        FOLDER "Tools/trace"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools/trace)
    add_dependencies(reflex_traces ${name})
endforeach()
