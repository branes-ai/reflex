// The scenario gallery (animation plan P4), at build time: every trace under
// src/traces/, its status (status.js), the run that produced it, a default
// set of plot panels derived from its channels, and the doc pages whose
// players show it. A new scenario joins the gallery with no edit here.

import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { traceStatus, failing } from './status.js';
import { playerId } from '../../components/trace/parts';

const traces = import.meta.glob('../../traces/*/*.json', { eager: true, import: 'default' });
// Written by check.mjs --report in the deploy workflow; absent in a local build.
const reports = import.meta.glob('../../run/report.json', { eager: true, import: 'default' });

/** The run behind the published traces, or undefined for the committed ones. */
export const report = Object.values(reports)[0];

// The blocks in the order of the docs map; a new block sorts after them.
const BLOCKS = {
  quadrotor: 'Quadrotor model',
  pid: 'PID controller',
  mixer: 'Mixer',
  cascade: 'Cascaded controller',
  filters: 'Gyro filters',
};
export const blockLabel = (dir) => BLOCKS[dir] ?? dir;
const blockRank = (dir) => {
  const i = Object.keys(BLOCKS).indexOf(dir);
  return i < 0 ? Object.keys(BLOCKS).length : i;
};

/** Every scenario, in the docs map's block order, then by name. */
export function scenarios() {
  const players = docPlayers();
  return Object.entries(traces)
    .map(([path, trace]) => {
      const [, dir, name] = /traces\/([^/]+)\/([^/]+)\.json$/.exec(path);
      const rel = `${dir}/${name}.json`;
      const status = traceStatus(trace);
      const runs = report
        ? Object.entries(report.runs).map(([label, r]) => ({ label, compiler: r.compiler, outcome: r.traces[rel] ?? (r.extra?.includes(rel) ? 'extra' : 'missing') }))
        : [];
      return {
        dir,
        name,
        trace,
        status,
        runs,
        failing: failing(status) || runs.some((r) => r.outcome !== 'match'),
        shownOn: (players.get(rel) ?? []).map((s) => ({ ...s, anchor: playerId(trace.provenance, s.title) })),
      };
    })
    .sort((a, b) => blockRank(a.dir) - blockRank(b.dir) || a.dir.localeCompare(b.dir) || a.name.localeCompare(b.name));
}

// --- The doc pages that play each trace -----------------------------------

const docsRoot = () => [resolve('../docs'), resolve('docs')].find((d) => existsSync(join(d, 'README.md')));

/**
 * Map from trace path (`<dir>/<name>.json`) to the players that show it:
 * { page, title, href (relative to the site's base), player }. Read from the
 * MDX sources: each `import x from '@site/traces/…'` and each
 * `<TracePlayer trace={x} title="…">`.
 */
function docPlayers() {
  const root = docsRoot();
  const found = new Map();
  if (!root) return found;
  const files = readdirSync(root, { recursive: true }).filter((f) => f.endsWith('.mdx') && !f.startsWith('sessions'));
  for (const file of files.map((f) => f.split('\\').join('/')).sort()) {
    const src = readFileSync(join(root, file), 'utf8');
    const imports = new Map([...src.matchAll(/^import (\w+) from '@site\/traces\/([\w/]+\.json)';$/gm)].map((m) => [m[1], m[2]]));
    if (imports.size === 0) continue;
    const page = /^title:\s*(.+)$/m.exec(src)?.[1].trim().replace(/^(['"])(.*)\1$/, '$2') ?? file;
    const href = `${file.replace(/\.mdx$/, '').replace(/(^|\/)index$/, '')}/`;
    // Players in fenced code are examples, not players.
    const body = src.replace(/^```[\s\S]*?^```/gm, '');
    for (const m of body.matchAll(/<TracePlayer\b([\s\S]*?)\/>/g)) {
      const rel = imports.get(/\btrace=\{(\w+)\}/.exec(m[1])?.[1]);
      const title = /\btitle="([^"]*)"/.exec(m[1])?.[1];
      if (!rel || !title) continue;
      if (!found.has(rel)) found.set(rel, []);
      found.get(rel).push({ page, title, href });
    }
  }
  return found;
}

// --- Default panels ---------------------------------------------------------

// Component labels by channel name (or name suffix, as in `mirror_position`).
const COMPONENTS = [
  [/(^|_)(position|velocity|body_rates)(_setpoint)?$/, ['x', 'y', 'z']],
  [/(^|_)rotor_(speeds|thrusts)(_setpoint)?$/, ['rotor 1', 'rotor 2', 'rotor 3', 'rotor 4']],
  [/(^|_)roll_pitch_yaw$/, ['roll', 'pitch', 'yaw']],
  [/(^|_)wrench$/, ['T', 'τx', 'τy', 'τz']],
];
const sameEntry = (a, b) => (Array.isArray(a) ? a.every((v, i) => v === b[i]) : a === b);
const widthOf = (data) => (Array.isArray(data[0]) ? data[0].length : 1);

/** Channels whose value never changes over the run, listed instead of plotted. */
export function constants(trace) {
  return Object.entries(trace.channels)
    .filter(([, data]) => data.every((e) => sameEntry(data[0], e)))
    .map(([name, data]) => ({ name, value: data[0] }));
}

/**
 * One panel per channel that changes, in the trace's order: a spectrogram
 * for `<x>_spectrum` with a `…bin_hz` channel, a line plot otherwise, with
 * `<x>_setpoint` drawn dashed on `<x>`'s panel. Quaternions (`…attitude`)
 * are left to the 3-D view and `roll_pitch_yaw`.
 */
export function derivePlots(trace) {
  const ch = trace.channels;
  const fixed = new Set(constants(trace).map((c) => c.name));
  const binHz = Object.keys(ch).find((n) => n.endsWith('bin_hz') && widthOf(ch[n]) === 1);
  const panels = [];
  for (const [name, data] of Object.entries(ch)) {
    if (fixed.has(name) || /(^|_)attitude$/.test(name)) continue;
    const base = name.replace(/_setpoint$/, '');
    if (base !== name && ch[base] && widthOf(ch[base]) === widthOf(data)) continue;  // drawn on its base's panel
    if (name.endsWith('_spectrum') && binHz) {
      panels.push({ kind: 'spectrogram', channel: name, binHz, title: name });
      continue;
    }
    const width = widthOf(data);
    const setpoint = Boolean(ch[`${name}_setpoint`]) && widthOf(ch[`${name}_setpoint`]) === width;
    const named = COMPONENTS.find(([re, l]) => re.test(name) && l.length === width)?.[1];
    // Four categorical colours; a wider channel's components share one (no cycling).
    const colors = width <= 4 ? undefined : Array(width).fill(1);
    const labels = named ?? (width === 1 ? [name] : Array.from({ length: width }, (_, k) => `[${k}]`));
    panels.push({ channel: name, title: name, labels, ...(colors && { colors }), ...(setpoint && { setpoint }) });
  }
  return panels;
}

/** The player's text description (ANI-10), from the trace itself. */
export function describe(trace, panels) {
  const p = trace.provenance;
  const n = trace.t.length;
  const duration = trace.t[n - 1];
  const checks = trace.checks ?? [];
  const plotted = panels.map((s) => s.channel).join(', ');
  return (
    `${duration} s of the ${p.block} scenario ${p.scenario.split('/').pop()}, ${n} samples` +
    `${p.dt_trace ? ` every ${p.dt_trace} s` : ''}. The panels plot every channel that changes: ${plotted}` +
    `${panels.some((s) => s.setpoint) ? ', with setpoints dashed' : ''}. ` +
    `${trace.events?.length ? `${trace.events.length} events are marked. ` : ''}` +
    (checks.length
      ? `Pass criteria: ${checks.map((c) => `${c.id} ${c.metric} ${c.comparison} ${c.limit}${c.unit !== '1' ? ` ${c.unit}` : ''}${c.expect === 'fail' ? ' (expected to fail)' : ''}`).join('; ')}.`
      : 'It has no pass criteria.')
  );
}
