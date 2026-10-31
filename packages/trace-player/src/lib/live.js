// A live scenario (Tier 2 of the animation standard): the player reruns the
// scenario in the reader's browser, from a WebAssembly module built from the
// same C++ as the native executable that wrote the recorded trace.
//
// The module is an Emscripten ES module (MODULARIZE, EXPORT_ES6) whose default
// export resolves to the module, with `ccall` and `UTF8ToString` exported and
// two C functions:
//   const char* reflex_run(const char* scenario, double p0, double p1, double p2);
//     the scenario's reflex-trace/1 JSON, or null on failure;
//   const char* reflex_error();  the reason for the last failure.
// reflex's tools/wasm/reflex_wasm.cpp is the reference implementation.

/**
 * One slider. `scale` converts the slider's display unit to the SI value the
 * module takes; `default` is the SI value the recorded trace was made with.
 * @typedef {{ name?: string, label: string, unit: string, scale: number,
 *   min: number, max: number, step?: number, default: number }} LiveParam
 */

/**
 * The `live` prop of TracePlayer: the name reflex_run() takes, up to three
 * parameters in the order it takes them, and the URL of the module's .mjs.
 * @typedef {{ scenario: string, params: LiveParam[], wasm: string }} LiveSpec
 */

/**
 * Runs `scenario` in a loaded module with SI parameter values; returns the parsed trace.
 * @param {any} module
 * @param {string} scenario
 * @param {number[]} values
 * @returns {unknown}
 */
export function runLive(module, scenario, values) {
  const p = [...values, 0, 0, 0].slice(0, 3);
  const ptr = module.ccall('reflex_run', 'number', ['string', 'number', 'number', 'number'], [scenario, ...p]);
  if (!ptr) throw new Error(module.ccall('reflex_error', 'string', [], []));
  return JSON.parse(module.UTF8ToString(ptr));
}
