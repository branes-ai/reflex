// trace_diff (BB-D3): equivalence of two reflex-trace/1 traces under a
// profile. Normative text: the "Equivalence profiles" section of
// docs/standards/animation. Reports the first sample and channel that fail.

import { reservedWidth } from './validate.js';

const RTOL = 1e-9;
const ATOL = 1e-9;
const ATOL_ROTOR_SPEEDS = 1e-6;
const CHECK_TOL = 1e-9;

/** Profiles: which parts are compared. */
export const PROFILES = {
  regen: { allChannels: true, events: true, checks: true, identity: true },
  wasm: { allChannels: false, events: false, checks: false, identity: false },
};

// Fields that say what a trace is; they must match exactly under `regen`
// (ANI-3). provenance.commit and provenance.compiler are excluded.
const IDENTITY_PROVENANCE = ['block', 'scenario', 'seed', 'dt_sim', 'dt_trace'];

function same(a, b) {
  return JSON.stringify(a) === JSON.stringify(b);
}

function within(a, b, atol, rtol) {
  return Math.abs(a - b) <= atol + rtol * Math.abs(b);
}

function atolFor(name) {
  return name === 'rotor_speeds' || name === 'rotor_speeds_setpoint' ? ATOL_ROTOR_SPEEDS : ATOL;
}

/** Rotation angle between two unit quaternions [w, x, y, z]; q and −q are equal. */
export function attitudeAngle(qa, qb) {
  // vec(qa⁻¹ ⊗ qb) for unit qa: conj(qa) ⊗ qb.
  const [aw, ax, ay, az] = qa;
  const [bw, bx, by, bz] = qb;
  const x = aw * bx - ax * bw - ay * bz + az * by;
  const y = aw * by + ax * bz - ay * bw - az * bx;
  const z = aw * bz - ax * by + ay * bx - az * bw;
  return 2 * Math.asin(Math.min(1, Math.hypot(x, y, z)));
}

/**
 * Compares `a` (candidate) against `b` (reference). Returns
 * { equivalent, failures: string[] }; the first failure of each kind is reported.
 */
export function diffTraces(a, b, profileName = 'regen') {
  const profile = PROFILES[profileName];
  if (!profile) throw new Error(`unknown profile ${profileName}`);
  const failures = [];
  const override = b.provenance.equivalence;

  if (profile.identity) {
    for (const key of ['schema', 'frames', 'units']) {
      if (!same(a[key] ?? null, b[key] ?? null)) failures.push(`${key}: differs`);
    }
    for (const key of IDENTITY_PROVENANCE) {
      if (!same(a.provenance[key] ?? null, b.provenance[key] ?? null)) {
        failures.push(`provenance.${key}: ${JSON.stringify(a.provenance[key])} vs ${JSON.stringify(b.provenance[key])}`);
      }
    }
    if (!same(a.provenance.equivalence ?? null, override ?? null)) failures.push('provenance.equivalence: differs');
  }

  // Rule 1: identical time bases (no resampling).
  if (!same(a.t, b.t)) {
    const k = a.t.findIndex((v, i) => v !== b.t[i]);
    failures.push(`t: differs (lengths ${a.t.length} and ${b.t.length}; first difference at index ${k < 0 ? Math.min(a.t.length, b.t.length) : k})`);
    return { equivalent: false, failures };
  }
  const horizon = override?.horizon ?? Infinity;

  // Rule 2: same compared channel names and shapes.
  const pick = (tr) =>
    Object.keys(tr.channels)
      .filter((name) => profile.allChannels || reservedWidth(name))
      .sort();
  const namesA = pick(a);
  const namesB = pick(b);
  if (!same(namesA, namesB)) {
    failures.push(`channels: [${namesA.join(', ')}] vs [${namesB.join(', ')}]`);
    return { equivalent: false, failures };
  }

  // Rule 3: per-component tolerances; attitude as one rotation angle.
  for (const name of namesB) {
    const ca = a.channels[name];
    const cb = b.channels[name];
    const tol = override?.tolerances?.[name] ?? {};
    const rtol = tol.rtol ?? RTOL;
    const atol = tol.atol ?? atolFor(name);
    for (let k = 0; k < b.t.length && b.t[k] <= horizon; k++) {
      const va = ca[k];
      const vb = cb[k];
      if (Array.isArray(va) !== Array.isArray(vb) || (Array.isArray(vb) && va.length !== vb.length)) {
        failures.push(`${name}: shape differs at t = ${b.t[k]}`);
        break;
      }
      if (name === 'attitude') {
        const angle = attitudeAngle(va, vb);
        if (!(angle <= atol)) {
          failures.push(`attitude: ${angle.toExponential(3)} rad apart at t = ${b.t[k]} (index ${k}), atol ${atol}`);
          break;
        }
        continue;
      }
      const xs = Array.isArray(vb) ? vb : [vb];
      const ys = Array.isArray(va) ? va : [va];
      const i = xs.findIndex((x, j) => !within(ys[j], x, atol, rtol));
      if (i >= 0) {
        failures.push(`${name}[${i}]: ${ys[i]} vs ${xs[i]} at t = ${b.t[k]} (index ${k}), atol ${atol}, rtol ${rtol}`);
        break;
      }
    }
  }

  // Rule 4: events match exactly.
  if (profile.events && !same(a.events ?? [], b.events ?? [])) failures.push('events: differ');

  // Rule 5: checks matched by id.
  if (profile.checks) {
    const byId = (tr) => new Map((tr.checks ?? []).map((c) => [c.id, c]));
    const ma = byId(a);
    const mb = byId(b);
    if (!same([...ma.keys()].sort(), [...mb.keys()].sort())) failures.push('checks: different sets of ids');
    for (const [id, cb] of mb) {
      const ca = ma.get(id);
      if (!ca) continue;
      for (const key of ['metric', 'unit', 'limit', 'comparison', 'pass']) {
        if (ca[key] !== cb[key]) failures.push(`check ${id}.${key}: ${JSON.stringify(ca[key])} vs ${JSON.stringify(cb[key])}`);
      }
      if (!within(ca.value, cb.value, CHECK_TOL, CHECK_TOL)) failures.push(`check ${id}.value: ${ca.value} vs ${cb.value}`);
    }
  }
  return { equivalent: failures.length === 0, failures };
}
