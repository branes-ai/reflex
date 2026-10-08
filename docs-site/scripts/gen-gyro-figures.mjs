// gen-gyro-figures.mjs — time-domain figures for the Mixed-Precision Gyro
// Filter page (issue #7).
//
// Reads the trace the C++ example writes and emits three self-contained SVGs
// into src/figures/gyro-lowpass/, which the page inlines (so the figures follow
// the docs' light/dark theme toggle):
//
//   input.svg   — the gyro signal decomposed: body motion, motor vibration,
//                 and their sum, as three panels on one shared scale (200 ms).
//   output.svg  — the filter output tracking the clean motion over the settled
//                 run (1.5 s), with the raw gyro input as context.
//   delay.svg   — a 60 ms zoom around a rising zero crossing: the output lags
//                 the motion by the filter's phase delay (~7.5 ms), and the
//                 residual 180 Hz ripple is visible.
//
// Regenerate (from the reflex repo root, after a gcc-debug build):
//   mkdir -p build/gyro-trace
//   REFLEX_DSP_TRACE_DIR=build/gyro-trace ./build/gcc-debug/tests/dsp_lowpass "*phase delay*"
//   node docs-site/scripts/gen-gyro-figures.mjs build/gyro-trace
//
// The SVGs are committed; the docs build does not run C++.

import { mkdirSync, readFileSync, writeFileSync, existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const traceDir = resolve(process.argv[2] ?? 'build/gyro-trace');
const tracePath = resolve(traceDir, 'gyro_lowpass.json');
if (!existsSync(tracePath)) {
  console.error(`no gyro_lowpass.json in ${traceDir}; run dsp_lowpass with REFLEX_DSP_TRACE_DIR set first`);
  process.exit(1);
}
const T = JSON.parse(readFileSync(tracePath, 'utf8'));
const outDir = resolve(here, '../src/figures/gyro-lowpass');
mkdirSync(outDir, { recursive: true });

const fs = T.fs_hz;
const n = T.motion.length;
const t = Array.from({ length: n }, (_, k) => k / fs);
const lagMs = T.measured_motion_lag_ms;
const r2 = T.response.find((r) => r.hz === T.motion_hz);
const r180 = T.response.find((r) => r.hz === T.vibration_hz);

// ── Style: tokens from the data-viz reference palette, both themes ─────────
// Series: slot 1 (blue) = body motion, slot 3 (aqua) = filter output, slot 2
// (orange) = vibration when shown alone. The raw gyro input is context, drawn
// in secondary ink. Dark values apply under the OS preference (unless the
// docs are toggled to light) and under the docs' dark toggle.
const STYLE = `
.gf { color-scheme: light;
  --surface: #fcfcfb; --ink: #0b0b0b; --ink2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7; --border: rgba(11,11,11,0.10);
  --motion: #2a78d6; --output: #1baf7a; --vib: #eb6834; --input: #8d8b85;
  font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }
@media (prefers-color-scheme: dark) { :root:where(:not([data-theme="light"])) .gf { color-scheme: dark;
  --surface: #1a1a19; --ink: #ffffff; --ink2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --motion: #3987e5; --output: #199e70; --vib: #d95926; --input: #8a887f; } }
:root[data-theme="dark"] .gf { color-scheme: dark;
  --surface: #1a1a19; --ink: #ffffff; --ink2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --motion: #3987e5; --output: #199e70; --vib: #d95926; --input: #8a887f; }
.gf .surface { fill: var(--surface); stroke: var(--border); }
.gf .grid { stroke: var(--grid); stroke-width: 1; }
.gf .axis { stroke: var(--axis); stroke-width: 1; }
.gf .tick { fill: var(--muted); font-size: 11px; font-variant-numeric: tabular-nums; }
.gf .title { fill: var(--ink); font-size: 14px; font-weight: 600; }
.gf .sub { fill: var(--ink2); font-size: 12px; }
.gf .label { fill: var(--ink2); font-size: 12px; }
.gf .note { fill: var(--ink); font-size: 12px; font-weight: 600; }
.gf .line { fill: none; stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }
.gf .line.thin { stroke-width: 1.25; }
.gf .s-motion { stroke: var(--motion); } .gf .k-motion { fill: var(--motion); }
.gf .s-output { stroke: var(--output); } .gf .k-output { fill: var(--output); }
.gf .s-vib { stroke: var(--vib); }       .gf .k-vib { fill: var(--vib); }
.gf .s-input { stroke: var(--input); }   .gf .k-input { fill: var(--input); }
.gf .bracket { stroke: var(--ink); stroke-width: 1.5; fill: none; }
.gf .dot { stroke: var(--surface); stroke-width: 2; }
.gf .xhair { stroke: var(--ink2); stroke-width: 1; visibility: hidden; }
.gf .tip .bg { fill: var(--surface); stroke: var(--border); }
.gf .tip text { fill: var(--ink); font-size: 12px; font-variant-numeric: tabular-nums; }
.gf .tip { visibility: hidden; }
.gf:focus { outline: 2px solid var(--motion); outline-offset: 2px; }
`;

const W = 720;
const PAD = { l: 56, r: 112, t: 86, b: 34 };
const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const fmt = (v, d) => v.toFixed(d);

function linePath(idx, values, sx, sy) {
  return idx.map((k, i) => `${i ? 'L' : 'M'}${sx(t[k]).toFixed(1)},${sy(values[k]).toFixed(1)}`).join('');
}
function range(t0, t1) {
  const a = Math.max(0, Math.round(t0 * fs));
  const b = Math.min(n - 1, Math.round(t1 * fs));
  return Array.from({ length: b - a + 1 }, (_, i) => a + i);
}

// One figure: one or more panels sharing the x range, each with one y scale.
// panels: [{ height, yMax, yStep, series: [{key, cls, name, values, thin}] }]
function figure({ file, title, subtitle, ariaLabel, t0, t1, xUnit, xTicks = 6, panels, legend, annotate }) {
  const idx = range(t0, t1);
  const plotW = W - PAD.l - PAD.r;
  const gap = 18;
  const totalH = panels.reduce((h, p) => h + p.height, 0) + gap * (panels.length - 1);
  const H = PAD.t + totalH + PAD.b;
  const sx = (x) => PAD.l + ((x - t0) / (t1 - t0)) * plotW;
  const xLabel = (x) => (xUnit === 'ms' ? fmt((x - t0) * 1000, 0) : fmt(x, 2));

  let body = '';
  let y = PAD.t;
  const hoverSeries = [];
  panels.forEach((p, pi) => {
    const top = y;
    const bot = y + p.height;
    const sy = (v) => top + ((p.yMax - v) / (2 * p.yMax)) * p.height;
    // y gridlines + ticks
    for (let v = -p.yMax; v <= p.yMax + 1e-9; v += p.yStep) {
      const yy = sy(v).toFixed(1);
      body += `<line class="${Math.abs(v) < 1e-9 ? 'axis' : 'grid'}" x1="${PAD.l}" x2="${PAD.l + plotW}" y1="${yy}" y2="${yy}"/>`;
      body += `<text class="tick" x="${PAD.l - 8}" y="${(+yy + 4).toFixed(1)}" text-anchor="end">${fmt(v, 1)}</text>`;
    }
    if (p.caption) body += `<text class="label" x="${PAD.l + 4}" y="${top + 14}">${esc(p.caption)}</text>`;
    // series
    p.series.forEach((s) => {
      body += `<path class="line s-${s.key}${s.thin ? ' thin' : ''}" d="${linePath(idx, s.values, sx, sy)}"/>`;
      if (s.label !== false) {
        // direct label at the right end, in ink, with a short line-key
        const ly = sy(s.values[idx[idx.length - 1]]) + (s.labelDy ?? 0);
        body += `<rect class="k-${s.key}" x="${PAD.l + plotW + 8}" y="${(ly - 1).toFixed(1)}" width="12" height="2" rx="1"/>`;
        body += `<text class="label" x="${PAD.l + plotW + 24}" y="${(ly + 4).toFixed(1)}">${esc(s.name)}</text>`;
      }
      hoverSeries.push({ name: s.name, key: s.key, panel: pi, v: idx.map((k) => +s.values[k].toFixed(4)) });
    });
    if (annotate && annotate.panel === pi) body += annotate.draw(sx, sy, top, bot);
    y = bot + gap;
  });
  // x axis ticks (bottom)
  for (let i = 0; i <= xTicks; i++) {
    const x = t0 + ((t1 - t0) * i) / xTicks;
    body += `<text class="tick" x="${sx(x).toFixed(1)}" y="${H - PAD.b + 18}" text-anchor="middle">${xLabel(x)}</text>`;
  }
  body += `<text class="tick" x="${PAD.l + plotW}" y="${H - 4}" text-anchor="end">${xUnit === 'ms' ? `time from ${fmt(t0, 2)} s [ms]` : 'time [s]'}</text>`;
  body += `<text class="tick" x="${PAD.l - 8}" y="${PAD.t - 8}" text-anchor="end">rad/s</text>`;

  // legend row (always present for >= 2 series)
  let lx = PAD.l;
  let legendSvg = '';
  legend.forEach(({ key, name }) => {
    legendSvg += `<rect class="k-${key}" x="${lx}" y="55" width="14" height="3" rx="1.5"/>`;
    legendSvg += `<text class="label" x="${lx + 20}" y="60">${esc(name)}</text>`;
    lx += 28 + name.length * 6.6;
  });

  const hover = {
    x0: PAD.l, w: plotW, top: PAD.t, bottom: PAD.t + totalH,
    t: idx.map((k) => +t[k].toFixed(4)), series: hoverSeries,
  };
  const svg = `<svg class="gf" viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="${esc(ariaLabel)}" tabindex="0" data-hover='${JSON.stringify(hover)}'>
<style>${STYLE}</style>
<rect class="surface" x="0.5" y="0.5" width="${W - 1}" height="${H - 1}" rx="8"/>
<text class="title" x="${PAD.l}" y="22">${esc(title)}</text>
<text class="sub" x="${PAD.l}" y="40">${esc(subtitle)}</text>
${legendSvg}
${body}
<line class="xhair" x1="0" x2="0" y1="${PAD.t}" y2="${PAD.t + totalH}"/>
<g class="tip"><rect class="bg" rx="4"/></g>
</svg>
`;
  writeFileSync(resolve(outDir, file), svg);
  console.log(`wrote ${file} (${(svg.length / 1024).toFixed(0)} KiB)`);
}

const motionSeries = { key: 'motion', name: 'body motion', values: T.motion };
const outputSeries = { key: 'output', name: 'filter output', values: T.output };
const inputSeries = { key: 'input', name: 'gyro input', values: T.input, thin: true };

// (a) decomposition — 200 ms around the rising zero crossing at t = 1 s
figure({
  file: 'input.svg',
  title: 'What the gyro measures',
  subtitle: `${T.motion_hz} Hz body motion + ${T.vibration_hz} Hz motor vibration = the signal the filter receives`,
  ariaLabel: 'Three panels on one scale over 200 ms: a slow 2 Hz body-motion sine, a fast 180 Hz vibration of amplitude 0.3 rad/s, and their sum.',
  t0: 0.9, t1: 1.1, xUnit: 's', xTicks: 4,
  legend: [{ key: 'motion', name: 'body motion' }, { key: 'vib', name: 'motor vibration' }, { key: 'input', name: 'gyro input (sum)' }],
  panels: [
    { height: 80, yMax: 0.8, yStep: 0.4, series: [{ ...motionSeries, label: false }], caption: `body motion, ${T.motion_hz} Hz` },
    { height: 80, yMax: 0.8, yStep: 0.4, series: [{ key: 'vib', name: 'vibration', values: T.vibration, label: false }], caption: `motor vibration, ${T.vibration_hz} Hz` },
    { height: 80, yMax: 0.8, yStep: 0.4, series: [{ ...inputSeries, thin: false, label: false }], caption: 'gyro input = motion + vibration' },
  ],
});

// (b) overview — the settled run
figure({
  file: 'output.svg',
  title: 'The filter recovers the motion',
  subtitle: `vibration cut ${fmt(-r180.gain_db, 1)} dB; the output follows the motion with a ${fmt(lagMs, 1)} ms lag (too small to see here, see the zoom)`,
  ariaLabel: 'Over 1.5 seconds the noisy gyro input is a thick band around a 2 Hz sine; the filter output sits on top of the clean motion.',
  t0: 0.5, t1: (n - 1) / fs, xUnit: 's',
  legend: [{ key: 'input', name: 'gyro input' }, { key: 'motion', name: 'body motion (truth)' }, { key: 'output', name: 'filter output' }],
  panels: [{ height: 200, yMax: 0.8, yStep: 0.4, series: [inputSeries, { ...motionSeries, labelDy: -8 }, { ...outputSeries, labelDy: 8 }] }],
});

// (c) zoom — the delay and the residual ripple
const tc = 1.0; // rising zero crossing of the motion (sin(2*pi*2*t) at t = 1 s)
figure({
  file: 'delay.svg',
  title: `The output lags the motion by ${fmt(lagMs, 1)} ms`,
  subtitle: `phase ${fmt(r2.phase_deg, 1)}° at ${T.motion_hz} Hz; residual ${T.vibration_hz} Hz ripple is ${fmt(r180.gain * 100, 1)}% of the vibration`,
  ariaLabel: `60 ms zoom around the rising zero crossing at t = 1 s: the filter output crosses zero ${fmt(lagMs, 1)} milliseconds after the body motion, and carries a small residual ripple.`,
  t0: 0.97, t1: 1.03, xUnit: 'ms',
  legend: [{ key: 'input', name: 'gyro input' }, { key: 'motion', name: 'body motion (truth)' }, { key: 'output', name: 'filter output' }],
  panels: [{
    height: 220, yMax: 0.5, yStep: 0.25,
    series: [inputSeries, { ...motionSeries, labelDy: -8 }, { ...outputSeries, labelDy: 8 }],
  }],
  annotate: {
    panel: 0,
    draw: (sx, sy) => {
      const x1 = sx(tc);
      const x2 = sx(tc + lagMs / 1000);
      const y0 = sy(0);
      const yb = sy(-0.33);
      return `<circle class="dot k-motion" cx="${x1.toFixed(1)}" cy="${y0.toFixed(1)}" r="4.5"/>`
        + `<circle class="dot k-output" cx="${x2.toFixed(1)}" cy="${y0.toFixed(1)}" r="4.5"/>`
        + `<path class="bracket" d="M${x1.toFixed(1)},${(y0 + 8).toFixed(1)}V${yb.toFixed(1)}M${x2.toFixed(1)},${(y0 + 8).toFixed(1)}V${yb.toFixed(1)}M${x1.toFixed(1)},${(yb - 4).toFixed(1)}H${x2.toFixed(1)}"/>`
        + `<text class="note" x="${((x1 + x2) / 2).toFixed(1)}" y="${(yb + 16).toFixed(1)}" text-anchor="middle">${fmt(lagMs, 1)} ms delay</text>`;
    },
  },
});
