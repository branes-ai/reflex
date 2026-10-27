// Build-time SVG plots for TracePlayer: one panel per PlotSpec, sharing the
// trace's time axis, in the same style as the SignalFigure charts. The client
// moves a time cursor across them and updates the legend readouts.

import { channelUnit } from '../../lib/trace/channels.js';

export interface PlotSpec {
  /** Channel to plot. */
  channel: string;
  /** Panel title; the unit is appended. */
  title: string;
  /** Components to draw (default: all). */
  components?: number[];
  /** Legend label per drawn component. */
  labels?: string[];
  /** Categorical color slot (1–4) per drawn component (default 1, 2, …). */
  colors?: number[];
  /** Overlay `<channel>_setpoint` as dashed lines of the same colors. */
  setpoint?: boolean;
  /** Fixed y-axis range [lo, hi], to zoom in on part of a signal; lines are clipped to it. Default: fit the data. */
  yRange?: [number, number];
  /** Further series from other channels, in the same units (for example a limit, or a second run). */
  series?: SeriesSpec[];
}

export interface SeriesSpec {
  channel: string;
  /** Component of a vector channel (default 0). */
  component?: number;
  label: string;
  /** Categorical color slot (1–4); default: the next after the main series. */
  color?: number;
  /** Draw dashed, as a reference rather than a measured signal. */
  dashed?: boolean;
}

export interface Trace {
  t: number[];
  channels: Record<string, (number | number[])[]>;
  units?: Record<string, string>;
  events?: { t: number; label: string }[];
}

export const W = 640;
const H = 128;
const X0 = 58;
const X1 = W - 12;
const TOP = 12;
const BOTTOM = H - 24;

function niceStep(span: number, target: number): number {
  const raw = span / target;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const f = raw / mag;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * mag;
}

function decimals(step: number): number {
  return Math.max(0, -Math.floor(Math.log10(step) + 1e-9));
}

function ticks(lo: number, hi: number, target: number) {
  const step = niceStep(hi - lo, target);
  const out: number[] = [];
  for (let v = Math.ceil(lo / step - 1e-9) * step; v <= hi + step * 1e-9; v += step) out.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return { step, values: out };
}


function component(entry: number | number[], i: number): number {
  return Array.isArray(entry) ? entry[i] : entry;
}

/** Display precision for readouts: enough digits to resolve the axis step. */
export function readoutDecimals(step: number): number {
  return decimals(step) + 1;
}

/**
 * Renders one panel. `key` makes the panel's SVG ids unique on the page; the
 * player passes the same key when it re-renders a panel for a live trace.
 */
export function renderPlot(trace: Trace, spec: PlotSpec, key = '') {
  const { t } = trace;
  const data = trace.channels[spec.channel];
  if (!data) throw new Error(`TracePlayer: no channel "${spec.channel}" in the trace`);
  const width = Array.isArray(data[0]) ? (data[0] as number[]).length : 1;
  const comps = spec.components ?? [...Array(width).keys()];
  const colors = spec.colors ?? comps.map((_, j) => j + 1);
  const setpoint = spec.setpoint ? trace.channels[`${spec.channel}_setpoint`] : undefined;
  if (spec.setpoint && !setpoint) throw new Error(`TracePlayer: no channel "${spec.channel}_setpoint"`);
  const unit = channelUnit(trace, spec.channel) ?? '';
  const extra = (spec.series ?? []).map((e, j) => {
    const series = trace.channels[e.channel];
    if (!series) throw new Error(`TracePlayer: no channel "${e.channel}" in the trace`);
    const u = channelUnit(trace, e.channel) ?? '';
    if (u !== unit) throw new Error(`TracePlayer: series "${e.channel}" is in ${u}, the plot in ${unit}`);
    return { ...e, data: series, component: e.component ?? 0, color: e.color ?? Math.min(4, comps.length + j + 1) };
  });

  // A loop, not Math.min(...values): spreading a long trace overflows the stack.
  let lo = Infinity;
  let hi = -Infinity;
  const extend = (series: (number | number[])[], c: number) => {
    for (const entry of series) {
      const v = component(entry, c);
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
  };
  for (const series of setpoint ? [data, setpoint] : [data]) comps.forEach((c) => extend(series, c));
  extra.forEach((e) => extend(e.data, e.component));
  if (spec.yRange) {
    [lo, hi] = spec.yRange;
    if (!(hi > lo)) throw new Error(`TracePlayer: plot "${spec.title}" has an empty yRange`);
  }
  const minSpan = Math.max(Math.abs(hi), Math.abs(lo), 1) * 0.1;
  if (hi - lo < minSpan * 1e-6) {
    lo -= minSpan / 2;
    hi += minSpan / 2;
  }
  const pad = spec.yRange ? 0 : (hi - lo) * 0.08;
  const yt = ticks(lo - pad, hi + pad, 3);
  lo = Math.min(lo - pad, yt.values[0]);
  hi = Math.max(hi + pad, yt.values[yt.values.length - 1]);

  const t0 = t[0];
  const t1 = t[t.length - 1];
  const x = (v: number) => X0 + ((v - t0) / (t1 - t0)) * (X1 - X0);
  const y = (v: number) => BOTTOM - ((v - lo) / (hi - lo)) * (BOTTOM - TOP);
  const xt = ticks(t0, t1, 6);
  const yd = decimals(yt.step);

  const grid = yt.values
    .map((v) => `<line class="grid" x1="${X0}" x2="${X1}" y1="${y(v).toFixed(1)}" y2="${y(v).toFixed(1)}"/>` +
      `<text class="tick" x="${X0 - 6}" y="${(y(v) + 4).toFixed(1)}" text-anchor="end">${v.toFixed(yd)}</text>`)
    .join('');
  const xticks = xt.values
    .map((v) => `<text class="tick" x="${x(v).toFixed(1)}" y="${BOTTOM + 15}" text-anchor="middle">${v.toFixed(decimals(xt.step))}</text>`)
    .join('');
  const events = (trace.events ?? [])
    .map((e) => `<line class="event" x1="${x(e.t).toFixed(1)}" x2="${x(e.t).toFixed(1)}" y1="${TOP}" y2="${BOTTOM}"/>`)
    .join('');
  const line = (series: (number | number[])[], c: number) =>
    series.map((e, k) => `${x(t[k]).toFixed(1)},${y(component(e, c)).toFixed(1)}`).join(' ');
  const lines = comps
    .map((c, j) =>
      (setpoint ? `<polyline class="line thin dashed s-${colors[j]}" points="${line(setpoint, c)}"/>` : '') +
      `<polyline class="line s-${colors[j]}" points="${line(data, c)}"/>`)
    .join('') +
    extra
      .map((e) => `<polyline class="line${e.dashed ? ' thin dashed' : ''} s-${e.color}" points="${line(e.data, e.component)}"/>`)
      .join('');

  // The legend (rendered as HTML by TracePlayer) doubles as the cursor readout.
  const labels = spec.labels ?? comps.map((c) => (width === 1 ? spec.title : `[${c}]`));
  const title = `${spec.title}${unit && unit !== '1' ? ` [${unit}]` : ''}`;
  const svg = `<svg class="tp-plot" viewBox="0 0 ${W} ${H}" aria-hidden="true" data-x0="${X0}" data-x1="${X1}">` +
    `<rect class="surface" x="0.5" y="0.5" width="${W - 1}" height="${H - 1}" rx="6"/>` +
    grid + events +
    `<line class="axis" x1="${X0}" x2="${X1}" y1="${BOTTOM}" y2="${BOTTOM}"/>` + xticks +
    `<text class="tick" x="6" y="${BOTTOM + 15}">t [s]</text>` +
    (spec.yRange
      ? `<clipPath id="tp-clip-${key}"><rect x="${X0}" y="${TOP}" width="${X1 - X0}" height="${BOTTOM - TOP}"/></clipPath>` +
        `<g clip-path="url(#tp-clip-${key})">${lines}</g>`
      : lines) +
    `<line class="tp-cursor" x1="${X0}" x2="${X0}" y1="${TOP - 2}" y2="${BOTTOM}"/>` +
    `</svg>`;
  return {
    title,
    svg,
    dashed: Boolean(setpoint),
    legend: [
      ...comps.map((_, j) => ({ label: labels[j], color: colors[j], dashed: false })),
      ...extra.map((e) => ({ label: e.label, color: e.color, dashed: Boolean(e.dashed) })),
    ],
    readout: {
      items: [
        ...comps.map((c) => ({ channel: spec.channel, component: c })),
        ...extra.map((e) => ({ channel: e.channel, component: e.component })),
      ],
      decimals: readoutDecimals(yt.step),
    },
  };
}
