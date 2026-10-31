// Reserved channels of reflex-trace/1: widths and units (docs/standards/
// animation, "Trace schema"). Kept free of the JSON Schema validator so the
// player can use it in the browser.

/** Width of each reserved channel. */
export const RESERVED = {
  position: 3,
  velocity: 3,
  attitude: 4,
  body_rates: 3,
  rotor_speeds: 4,
  rotor_thrusts: 4,
  wrench: 4,
};

/** Units of the reserved channels, for display. */
export const RESERVED_UNITS = {
  position: 'm',
  velocity: 'm/s',
  attitude: '1',
  body_rates: 'rad/s',
  rotor_speeds: 'rad/s',
  rotor_thrusts: 'N',
  wrench: 'N, N·m',
};

const ANGLE_SETPOINTS = new Set(['roll_setpoint', 'pitch_setpoint', 'yaw_setpoint']);

/** Width of a reserved channel (including `<quantity>_setpoint`), or 0. */
export function reservedWidth(name) {
  if (name in RESERVED) return RESERVED[name];
  if (ANGLE_SETPOINTS.has(name)) return 1;
  const m = /^(.+)_setpoint$/.exec(name);
  return m && m[1] in RESERVED ? RESERVED[m[1]] : 0;
}

/** Unit of any channel: reserved, or declared in `units`. */
export function channelUnit(trace, name) {
  if (ANGLE_SETPOINTS.has(name)) return 'rad';
  const base = name.replace(/_setpoint$/, '');
  if (reservedWidth(name)) return RESERVED_UNITS[base];
  return trace.units?.[name];
}
