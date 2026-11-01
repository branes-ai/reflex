// reflex-trace/1 validation: the JSON Schema (structure and types) plus the
// semantic rules it cannot express. Normative text: the "Trace schema" section
// of docs/standards/animation. Used at build time by TracePlayer (which refuses
// a trace that fails) and by scripts/trace/check.mjs in CI.

import Ajv2020 from 'ajv/dist/2020.js';
import schema from '../../schemas/reflex-trace-1.schema.json' with { type: 'json' };

// The channel tables live in channels.js, which the browser loads without
// ajv; they are re-exported here for the build-time callers.
export { RESERVED, RESERVED_UNITS, reservedWidth, channelUnit } from './channels.js';
import { reservedWidth } from './channels.js';

const ajv = new Ajv2020({ allErrors: true, strict: true });
const checkSchema = ajv.compile(schema);

/** Unit-norm tolerance on attitude quaternions. */
export const ATTITUDE_NORM_TOL = 1e-6;

/**
 * Validates a parsed trace. Returns a list of error strings; empty means valid.
 */
export function validateTrace(trace) {
  if (!checkSchema(trace)) {
    return checkSchema.errors.map((e) => `schema: ${e.instancePath || '/'} ${e.message}`);
  }
  const errors = [];
  const { t, channels, provenance } = trace;
  const n = t.length;

  // Time: starts at 0, strictly increasing, uniform at dt_trace when given.
  if (t[0] !== 0) errors.push(`t: starts at ${t[0]}, not 0`);
  for (let k = 1; k < n; k++) {
    if (!(t[k] > t[k - 1])) {
      errors.push(`t: not strictly increasing at index ${k}`);
      break;
    }
  }
  const dt = provenance.dt_trace;
  if (dt !== undefined) {
    for (let k = 0; k < n; k++) {
      if (Math.abs(t[k] - k * dt) > 1e-9 * dt) {
        errors.push(`t: index ${k} is ${t[k]}, not ${k} × dt_trace = ${k * dt}`);
        break;
      }
    }
  }

  // Channels: one entry per sample, fixed width, reserved widths, units.
  for (const [name, data] of Object.entries(channels)) {
    if (data.length !== n) {
      errors.push(`channel ${name}: ${data.length} entries for ${n} samples`);
      continue;
    }
    const width = Array.isArray(data[0]) ? data[0].length : 1;
    const scalar = !Array.isArray(data[0]);
    for (let k = 0; k < n; k++) {
      const entry = data[k];
      if (scalar ? Array.isArray(entry) : !Array.isArray(entry) || entry.length !== width) {
        errors.push(`channel ${name}: entry ${k} does not have width ${width}`);
        break;
      }
    }
    const reserved = reservedWidth(name);
    if (reserved) {
      if (width !== reserved || (reserved > 1 && scalar)) {
        errors.push(`channel ${name}: reserved width is ${reserved}, found ${width}`);
      }
      if (trace.units && name in trace.units) {
        errors.push(`units: reserved channel ${name} must not declare a unit`);
      }
    } else if (!trace.units?.[name]) {
      errors.push(`channel ${name}: not reserved, so it needs a unit in "units"`);
    }
    if (name === 'attitude' && width === 4) {
      for (let k = 0; k < n; k++) {
        const norm = Math.hypot(...data[k]);
        if (Math.abs(norm - 1) > ATTITUDE_NORM_TOL) {
          errors.push(`channel attitude: entry ${k} has norm ${norm}`);
          break;
        }
      }
    }
  }
  for (const name of Object.keys(trace.units ?? {})) {
    if (!(name in channels)) errors.push(`units: ${name} is not a channel`);
  }

  // Events within the time range.
  for (const e of trace.events ?? []) {
    if (!(e.t >= t[0] && e.t <= t[n - 1])) errors.push(`event "${e.label}": t = ${e.t} outside [${t[0]}, ${t[n - 1]}]`);
  }

  // Scene: unique ids, unit quaternions, a tree drawn on a cylinder, and pose
  // channels that exist with the pose's width (unit norm for an attitude).
  const unit = (q) => Math.abs(Math.hypot(...q) - 1) <= ATTITUDE_NORM_TOL;
  const entityIds = new Set();
  for (const e of trace.scene?.entities ?? []) {
    const where = `scene entity ${e.id}`;
    if (entityIds.has(e.id)) errors.push(`${where}: duplicate id`);
    entityIds.add(e.id);
    for (const part of e.shape.type === 'compound' ? e.shape.parts : []) {
      if (part.offset.attitude && !unit(part.offset.attitude)) errors.push(`${where}: a part's attitude is not a unit quaternion`);
    }
    if (e.visual?.kind === 'tree' && e.shape.type !== 'cylinder') errors.push(`${where}: a tree is drawn on a cylinder, not a ${e.shape.type}`);
    for (const [key, width] of [['position', 3], ['attitude', 4]]) {
      const v = e.pose[key];
      if (v === undefined) continue;
      if (typeof v !== 'string') {
        if (key === 'attitude' && !unit(v)) errors.push(`${where}: its attitude is not a unit quaternion`);
        continue;
      }
      const data = channels[v];
      if (!data) {
        errors.push(`${where}: its ${key} channel ${v} is not a channel`);
      } else if (!Array.isArray(data[0]) || data[0].length !== width) {
        errors.push(`${where}: its ${key} channel ${v} has width ${Array.isArray(data[0]) ? data[0].length : 1}, not ${width}`);
      } else if (key === 'attitude') {
        const k = data.findIndex((q) => !unit(q));
        if (k >= 0) errors.push(`${where}: its attitude channel ${v} has a non-unit entry at ${k}`);
      }
    }
  }

  // Checks: unique ids, and pass consistent with value, limit, and comparison.
  const ids = new Set();
  for (const c of trace.checks ?? []) {
    if (ids.has(c.id)) errors.push(`check ${c.id}: duplicate id`);
    ids.add(c.id);
    const pass = c.comparison === '<=' ? c.value <= c.limit : c.value >= c.limit;
    if (pass !== c.pass) errors.push(`check ${c.id}: pass is ${c.pass}, but ${c.value} ${c.comparison} ${c.limit} is ${pass}`);
  }
  return errors;
}
