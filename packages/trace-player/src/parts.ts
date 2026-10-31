// HTML for the parts of TracePlayer that depend on the trace: the event list,
// the checks, and the provenance footer. Rendered at build time by the
// component and again in the browser when a live widget replaces the trace,
// so both paths produce the same markup.

import { checkStatus, STATUS_LABEL } from './lib/status.js';

export interface TraceParts {
  provenance: {
    commit: string;
    block: string;
    scenario: string;
    seed: number;
    compiler: string;
    dt_sim: number;
    dt_trace?: number;
  };
  schema: string;
  t: number[];
  events?: { t: number; label: string }[];
  checks?: { id: string; metric: string; unit: string; value: number; limit: number; comparison: string; pass: boolean; expect?: string }[];
}

const DEFAULT_REPO = 'https://github.com/branes-ai/reflex';
const NOTE = 'checked by CI against a fresh run';

/**
 * What the footer links to and says about the trace: `repo`, the GitHub
 * repository that holds the scenario and the commit (default reflex), and
 * `note`, how the trace is kept (default: checked by CI against a fresh run).
 */
export interface FooterOptions {
  repo?: string;
  note?: string;
}

export function escapeHtml(s: string): string {
  return s.replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]!);
}

/** A check value or limit: 4 significant digits, or exponent form when tiny or huge. */
export function fmt(v: number): string {
  return v === 0 || (Math.abs(v) >= 1e-3 && Math.abs(v) < 1e4) ? String(Number(v.toPrecision(4))) : v.toExponential(2);
}

/** An event time: to 10 ms, or to 1 ms in a trace shorter than a second. */
export function eventTime(trace: TraceParts, t: number): string {
  return t.toFixed(trace.t[trace.t.length - 1] < 1 ? 3 : 2);
}

export function renderEvents(trace: TraceParts): string {
  if (!trace.events?.length) return '';
  const items = trace.events
    .map((e) => `<li><button type="button" data-seek="${e.t}">t = ${eventTime(trace, e.t)} s</button> ${escapeHtml(e.label)}</li>`)
    .join('');
  return `<ol class="tp-events">${items}</ol>`;
}

export function renderChecks(trace: TraceParts): string {
  if (!trace.checks?.length) return '';
  const items = trace.checks
    .map((c) => {
      const status = checkStatus(c);
      return `<li class="${status}"><span class="tp-badge">${c.pass ? '✓' : '✗'} ${STATUS_LABEL[status]}</span>` +
      `<b>${escapeHtml(c.id)}</b> ${escapeHtml(c.metric)} = ${fmt(c.value)}${c.unit !== '1' ? ` ${escapeHtml(c.unit)}` : ''} ` +
      `(${escapeHtml(c.comparison)} ${fmt(c.limit)}), computed by the scenario harness</li>`;
    })
    .join('');
  return `<ul class="tp-checks">${items}</ul>`;
}

/**
 * The player's element id, unique on a page even when several players show
 * one trace: the title, which differs between players, is hashed into it
 * (FNV-1a, 32 bits). The scenario gallery links to players by it.
 */
export function playerId(provenance: { block: string; scenario: string }, title: string): string {
  let hash = 0x811c9dc5;
  for (const c of title) hash = Math.imul(hash ^ c.codePointAt(0)!, 0x01000193) >>> 0;
  return `tp-${provenance.block.replace(/\W+/g, '-')}-${provenance.scenario.split('/').pop()?.replace(/\W+/g, '-')}-${hash.toString(36)}`;
}

/** The footer's contents. `live` marks a trace computed in the reader's browser. */
export function renderProvenance(trace: TraceParts, live = false, footer: FooterOptions = {}): string {
  const p = trace.provenance;
  const REPO = (footer.repo ?? DEFAULT_REPO).replace(/\/$/, '');
  const code = (s: string) => `<code>${escapeHtml(s)}</code>`;
  const hash = /^[0-9a-f]{7,40}$/.test(p.commit);
  const commit = hash ? `<a href="${REPO}/commit/${p.commit}">${code(p.commit)}</a>` : code(p.commit);
  // The scenario as it was at the recorded commit; without a hash, the default branch's.
  const ref = hash ? p.commit : 'HEAD';
  const spans = [
    `Block ${code(p.block)}`,
    `Scenario <a href="${REPO}/blob/${ref}/${escapeHtml(p.scenario)}">${code(p.scenario)}</a>`,
    `Seed ${p.seed}`,
    `dt<sub>sim</sub> ${p.dt_sim} s · dt<sub>trace</sub> ${p.dt_trace ?? 'non-uniform'} s`,
    escapeHtml(p.compiler),
    `${live ? 'Built from' : 'Generated at'} ${commit}`,
    live
      ? `Format ${code(trace.schema)}, computed in this browser by the same C++ compiled to WebAssembly`
      : `Format ${code(trace.schema)}, ${escapeHtml(footer.note ?? NOTE)}`,
  ];
  return spans.map((s) => `<span>${s}</span>`).join('');
}
