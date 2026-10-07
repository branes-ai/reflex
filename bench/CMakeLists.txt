# bench/ — micro-benchmarks for the control components.
#
# Each .cpp here is a standalone executable that reports per-update latency.
# They are built by default (REFLEX_BUILD_BENCH) so they keep compiling, but are
# not registered with CTest: timings are machine-dependent and do not gate CI.

file(GLOB BENCH_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(source ${BENCH_SRC})
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE branes::reflex)
    target_compile_features(${name} PRIVATE cxx_std_20)
    reflex_set_warnings(${name})
    set_target_properties(${name} PROPERTIES FOLDER "Bench")
endforeach()
