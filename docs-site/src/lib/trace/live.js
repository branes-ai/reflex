// The live scenarios: Tier 2 of docs/standards/animation. Each runs in the
// browser from the WebAssembly module (tools/wasm/reflex_wasm.cpp), with the
// same C++ as the native trace executable that wrote its committed trace.
//
// For each scenario: the committed trace it starts from, and up to three
// parameters, in the order reflex_run() takes them. Ranges match the checks in
// the scenario's header (ANI-9: they stay inside the block's validity
// envelope). `scale` converts the slider's display unit to the SI value the
// module takes; `default` is the SI value the committed trace was made with.

export const LIVE = {
  motor_lag: {
    trace: 'quadrotor/motor_lag.json',
    params: [
      { name: 'tau', label: 'Motor time constant τ', unit: 'ms', scale: 1e-3, min: 5, max: 100, step: 1, default: 0.02 },
    ],
  },
  tumble: {
    trace: 'quadrotor/tumble.json',
    params: [
      { name: 'jx', label: 'J_x', unit: '×10⁻³ kg·m²', scale: 1e-3, min: 0.5, max: 10, step: 0.5, default: 0.002 },
      { name: 'jy', label: 'J_y', unit: '×10⁻³ kg·m²', scale: 1e-3, min: 0.5, max: 10, step: 0.5, default: 0.004 },
      { name: 'jz', label: 'J_z', unit: '×10⁻³ kg·m²', scale: 1e-3, min: 0.5, max: 10, step: 0.5, default: 0.006 },
    ],
  },
  roll_step: {
    trace: 'cascade/roll_step.json',
    params: [
      { name: 'attitude_gain', label: 'Attitude gain K', unit: '1/s', scale: 1, min: 2, max: 30, step: 1, default: 10 },
      { name: 'rate_gain', label: 'Rate gain k_p', unit: '1/s', scale: 1, min: 10, max: 150, step: 5, default: 50 },
    ],
  },
  pid_terms: {
    trace: 'pid/terms.json',
    params: [
      { name: 'kp', label: 'Proportional gain k_p', unit: '1/s²', scale: 1, min: 2, max: 40, step: 1, default: 16 },
      { name: 'ki', label: 'Integral gain k_i', unit: '1/s³', scale: 1, min: 0, max: 30, step: 1, default: 8 },
      { name: 'kd', label: 'Derivative gain k_d', unit: '1/s', scale: 1, min: 0, max: 20, step: 0.5, default: 8 },
    ],
  },
  pid_derivative_filter: {
    trace: 'pid/derivative_filter.json',
    params: [
      { name: 'tau', label: 'Derivative filter τ', unit: 'ms', scale: 1e-3, min: 0.5, max: 500, step: 0.5, default: 0.005 },
    ],
  },
  pid_windup: {
    trace: 'pid/windup.json',
    params: [
      { name: 'ki', label: 'Integral gain k_i', unit: '1/s', scale: 1, min: 1, max: 30, step: 1, default: 10 },
      { name: 'hold', label: 'Time at the unreachable setpoint', unit: 's', scale: 1, min: 0.5, max: 3, step: 0.1, default: 2 },
    ],
  },
  mixer_priority: {
    trace: 'mixer/priority.json',
    params: [
      { name: 'throttle', label: 'Throttle, F / 4·T_max', unit: '%', scale: 0.01, min: 5, max: 100, step: 5, default: 0.75 },
      { name: 'yaw', label: 'Yaw torque τz', unit: 'N·m', scale: 1, min: 0, max: 0.3, step: 0.01, default: 0.1 },
    ],
  },
  mixer_clip_vs_priority: {
    trace: 'mixer/clip_vs_priority.json',
    params: [
      { name: 'throttle', label: 'Throttle, F / 4·T_max', unit: '%', scale: 0.01, min: 0, max: 100, step: 5, default: 1 },
    ],
  },
};

/** Runs `scenario` in a loaded module with SI parameter values; returns the parsed trace. */
export function runLive(module, scenario, values) {
  const p = [...values, 0, 0, 0].slice(0, 3);
  const ptr = module.ccall('reflex_run', 'number', ['string', 'number', 'number', 'number'], [scenario, ...p]);
  if (!ptr) throw new Error(module.ccall('reflex_error', 'string', [], []));
  return JSON.parse(module.UTF8ToString(ptr));
}
