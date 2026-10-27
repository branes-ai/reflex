# reflex_wasm: the live-widget scenarios as a WebAssembly ES module
# (reflex.mjs + reflex.wasm). Built with Emscripten by scripts/build-wasm.sh;
# see reflex_wasm.cpp for the C ABI.

set(REFLEX_WASM_COMMIT "unknown" CACHE STRING "Commit recorded in the module's traces")

add_executable(reflex_wasm reflex_wasm.cpp)
target_link_libraries(reflex_wasm PRIVATE branes::reflex branes::reflex_trace)
target_include_directories(reflex_wasm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../trace/scenarios)
target_compile_definitions(reflex_wasm PRIVATE REFLEX_WASM_COMMIT="${REFLEX_WASM_COMMIT}")
reflex_set_warnings(reflex_wasm)
# Native Wasm exceptions: the trace writer reports invalid parameters by
# throwing, and reflex_run turns that into an error string.
target_compile_options(reflex_wasm PRIVATE -fwasm-exceptions)
target_link_options(reflex_wasm PRIVATE
    -fwasm-exceptions
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createReflex
    -sENVIRONMENT=web,node
    -sALLOW_MEMORY_GROWTH=1
    -sFILESYSTEM=0
    -sEXPORTED_FUNCTIONS=_reflex_run,_reflex_error,_reflex_version
    -sEXPORTED_RUNTIME_METHODS=ccall,UTF8ToString)
set_target_properties(reflex_wasm PROPERTIES
    OUTPUT_NAME reflex
    SUFFIX .mjs
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/wasm)
