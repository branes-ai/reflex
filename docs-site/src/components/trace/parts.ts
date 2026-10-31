// HTML for the parts of TracePlayer that depend on the trace: the event list,
// the checks, and the provenance footer. Rendered at build time by the
// component and again in the browser when a live widget replaces the trace,
// so both paths produce the same markup.

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
  checks?: { id: string; metric: string; unit: string; value: number; limit: number; comparison: string; pass: boolean }[];
}

const REPO = 'https://github.com/branes-ai/reflex';

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
    .map((c) =>
      `<li class="${c.pass ? 'pass' : 'fail'}"><span class="tp-badge">${c.pass ? '✓ pass' : '✗ fail'}</span>` +
      `<b>${escapeHtml(c.id)}</b> ${escapeHtml(c.metric)} = ${fmt(c.value)}${c.unit !== '1' ? ` ${escapeHtml(c.unit)}` : ''} ` +
      `(${escapeHtml(c.comparison)} ${fmt(c.limit)}), computed by the scenario harness</li>`)
    .join('');
  return `<ul class="tp-checks">${items}</ul>`;
}

/** The footer's contents. `live` marks a trace computed in the reader's browser. */
export function renderProvenance(trace: TraceParts, live = false): string {
  const p = trace.provenance;
  const code = (s: string) => `<code>${escapeHtml(s)}</code>`;
  const commit = /^[0-9a-f]{7,40}$/.test(p.commit) ? `<a href="${REPO}/commit/${p.commit}">${code(p.commit)}</a>` : code(p.commit);
  const spans = [
    `Block ${code(p.block)}`,
    `Scenario <a href="${REPO}/blob/main/${escapeHtml(p.scenario)}">${code(p.scenario)}</a>`,
    `Seed ${p.seed}`,
    `dt<sub>sim</sub> ${p.dt_sim} s · dt<sub>trace</sub> ${p.dt_trace ?? 'non-uniform'} s`,
    escapeHtml(p.compiler),
    `${live ? 'Built from' : 'Generated at'} ${commit}`,
    live
      ? `Format ${code(trace.schema)}, computed in this browser by the same C++ compiled to WebAssembly`
      : `Format ${code(trace.schema)}, checked by CI against a fresh run`,
  ];
  return spans.map((s) => `<span>${s}</span>`).join('');
}
