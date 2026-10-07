# sdk/ — reflex control & safety components (header-only).
#
# `branes::reflex` is an INTERFACE target carrying the include path and the
# C++20 requirement. Controllers are templated on the scalar type, so the same
# code runs in double, float, or custom arithmetic (e.g. Universal posits).

add_library(branes_reflex INTERFACE)
add_library(branes::reflex ALIAS branes_reflex)

target_include_directories(branes_reflex INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(branes_reflex INTERFACE cxx_std_20)
