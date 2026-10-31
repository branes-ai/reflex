# reflex-trace/1 writer (header-only), for the scenario executables that
# produce the traces the docs-site player renders. Defined for consumers too
# (racing-drone exports its mission replays with it), whatever
# REFLEX_BUILD_TOOLS says; it needs only the standard library.
add_library(reflex_trace INTERFACE)
add_library(branes::reflex_trace ALIAS reflex_trace)
target_include_directories(reflex_trace INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(reflex_trace INTERFACE cxx_std_20)
