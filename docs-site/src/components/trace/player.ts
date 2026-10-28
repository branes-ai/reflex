// Client side of TracePlayer: the time base, controls, keyboard, plot cursors
// and readouts, the lazily loaded three.js vehicle view, and, for a live
// widget, the sliders that rerun the scenario in WebAssembly.
//
// Rules from the animation standard (docs/standards/animation):
//   - starts paused, and only a reader action plays it (ANI-10);
//   - the rate is always on screen (ANI-5);
//   - sample-and-hold: the cursor sits on a recorded sample, never between
//     samples, so nothing on screen is computed here (ANI-1). A live trace
//     comes from the scenario's own C++, compiled to WebAssembly.

interface Readout {
  items: { channel: string; component: number }[];
  decimals: number;
  bars?: BarsReadout;
}

type Trace = TraceParts & {
  t: number[];
  channels: Record<string, (number | number[])[]>;
  units?: Record<string, string>;
  vehicle?: { arm_length?: number };
};

interface LiveParam {
  label: string;
  unit: string;
  scale: number;
  min: number;
  max: number;
  default: number;
}

interface LiveConfig {
  scenario: string;
  params: LiveParam[];
  plots: PanelSpec[];
  key: string;
  wasm: string;
}

interface Payload {
  trace: Trace;
  config: { thrustScale: number; panels: Readout[]; ghost?: GhostSpec; live?: LiveConfig };
}

export interface ViewHandle {
  show(index: number): void;
  setCamera(mode: string): void;
  setTrace(trace: Trace): void;
}

import type { GhostSpec } from './vehicle-view';
import { barGeometry, renderPanel, type BarsReadout, type PanelSpec } from './plot';
import { renderChecks, renderEvents, renderProvenance, type TraceParts } from './parts';
import { runLive } from '../../lib/trace/live.js';

// One WebAssembly module per page, loaded on the first slider move.
let modulePromise: Promise<any> | null = null;
function loadModule(url: string) {
  modulePromise ??= import(/* @vite-ignore */ url)
    .then((m) => m.default())
    .catch((err) => {
      modulePromise = null;  // let a later slider move try again
      throw err;
    });
  return modulePromise;
}

const reducedMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const SPEEDS = [0.1, 0.25, 0.5, 1];

function attach(root: HTMLElement) {
  const payload: Payload = JSON.parse(root.querySelector('script[data-trace]')!.textContent!);
  const { config } = payload;
  const recorded = payload.trace;
  let trace = recorded;
  let t = trace.t;
  let n = t.length;
  const play = root.querySelector<HTMLButtonElement>('[data-play]')!;
  const icon = root.querySelector<SVGPathElement>('[data-play-icon]')!;
  const scrub = root.querySelector<HTMLInputElement>('[data-scrub]')!;
  const time = root.querySelector<HTMLElement>('[data-time]')!;
  const speed = root.querySelector<HTMLSelectElement>('[data-speed]')!;
  const badge = root.querySelector<HTMLElement>('[data-rate-badge]')!;
  const eventBox = root.querySelector<HTMLElement>('[data-event]');
  const panels = [...root.querySelectorAll<HTMLElement>('[data-panel]')].map((el, i) => ({
    host: el.querySelector<HTMLElement>('[data-svg]')!,
    svg: el.querySelector<SVGSVGElement>('svg')!,
    cursor: el.querySelector<SVGLineElement>('.tp-cursor'),
    values: [...el.querySelectorAll<HTMLElement>('[data-readout]')],
    readout: config.panels[i],
  }));
  const parts = {
    events: root.querySelector<HTMLElement>('[data-events]')!,
    checks: root.querySelector<HTMLElement>('[data-checks]')!,
    prov: root.querySelector<HTMLElement>('[data-prov]')!,
  };

  let index = 0;
  let playing = false;
  let rate = 1;
  let simTime = 0;
  let last = 0;
  let view: ViewHandle | null = null;

  const indexAt = (time: number) => {
    // Largest k with t[k] ≤ time.
    let lo = 0;
    let hi = n - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (t[mid] <= time) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  };

  function render() {
    const tk = t[index];
    const label = `t = ${tk.toFixed(3)} s`;
    time.textContent = `${label} / ${t[n - 1].toFixed(3)} s`;
    scrub.value = String(index);
    scrub.setAttribute('aria-valuetext', label);
    for (const p of panels) {
      if (p.readout.bars) {
        showBars(p.svg, trace, index, p.readout.bars);
        continue;
      }
      const x0 = Number(p.svg.dataset.x0);
      const x1 = Number(p.svg.dataset.x1);
      const x = x0 + ((tk - t[0]) / (t[n - 1] - t[0])) * (x1 - x0);
      p.cursor!.setAttribute('x1', String(x));
      p.cursor!.setAttribute('x2', String(x));
      p.readout.items.forEach(({ channel, component: c }, j) => {
        const entry = trace.channels[channel][index];
        const v = Array.isArray(entry) ? entry[c] : entry;
        const text = v.toFixed(p.readout.decimals);
        p.values[j].textContent = Number(text) === 0 ? (0).toFixed(p.readout.decimals) : text;  // no "-0.00"
      });
    }
    if (eventBox) {
      const passed = (trace.events ?? []).filter((e) => e.t <= tk);
      eventBox.textContent = passed.length ? `t = ${passed[passed.length - 1].t.toFixed(2)} s: ${passed[passed.length - 1].label}` : '';
    }
    view?.show(index);
  }

  function seek(k: number) {
    index = Math.max(0, Math.min(n - 1, k));
    simTime = t[index];
    render();
  }

  function setPlaying(on: boolean) {
    playing = on;
    play.setAttribute('aria-label', on ? 'Pause' : 'Play');
    icon.setAttribute('d', on ? 'M4 2.5h3v11H4zM9 2.5h3v11H9z' : 'M4 2.5v11l9-5.5z');
    if (on) {
      if (index >= n - 1) seek(0);
      last = performance.now();
      requestAnimationFrame(tick);
    }
  }

  function tick(now: number) {
    if (!playing) return;
    simTime += ((now - last) / 1000) * rate;
    last = now;
    if (simTime >= t[n - 1]) {
      seek(n - 1);
      setPlaying(false);
      return;
    }
    const k = indexAt(simTime);
    if (k !== index) {
      index = k;
      render();
    }
    requestAnimationFrame(tick);
  }

  function setRate(r: number) {
    rate = r;
    speed.value = String(r);
    badge.textContent = r === 1 ? '1× real time' : `${r}× real time (slowed)`;
  }

  play.addEventListener('click', () => setPlaying(!playing));
  scrub.addEventListener('input', () => {
    setPlaying(false);
    seek(Number(scrub.value));
  });
  speed.addEventListener('change', () => setRate(Number(speed.value)));
  // Event buttons are re-rendered with a live trace, so listen on the list.
  parts.events.addEventListener('click', (ev) => {
    const b = (ev.target as HTMLElement).closest<HTMLButtonElement>('[data-seek]');
    if (!b) return;
    setPlaying(false);
    seek(indexAt(Number(b.dataset.seek)));
  });

  // Scrubbing on any plot. The listeners sit on the panel's SVG host, which
  // outlives the SVG when a live trace re-renders it.
  for (const p of panels) {
    if (p.readout.bars) continue;  // no time axis to scrub along
    const toIndex = (ev: PointerEvent) => {
      const ctm = p.svg.getScreenCTM();
      if (!ctm) return index;
      const x = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(ctm.inverse()).x;
      const x0 = Number(p.svg.dataset.x0);
      const x1 = Number(p.svg.dataset.x1);
      return indexAt(t[0] + ((x - x0) / (x1 - x0)) * (t[n - 1] - t[0]));
    };
    p.host.addEventListener('pointerdown', (ev) => {
      setPlaying(false);
      p.host.setPointerCapture(ev.pointerId);
      seek(toIndex(ev));
    });
    p.host.addEventListener('pointermove', (ev) => {
      if (p.host.hasPointerCapture(ev.pointerId)) seek(toIndex(ev));
    });
  }

  /** Replaces the trace: plots, events, checks, provenance, and the 3-D view. */
  function load(next: Trace, live: boolean) {
    trace = next;
    t = trace.t;
    n = t.length;
    scrub.max = String(n - 1);
    if (config.live) {
      panels.forEach((p, i) => {
        const r = renderPanel(trace, config.live!.plots[i], `${config.live!.key}-${i}`);
        p.host.innerHTML = r.svg;
        p.svg = p.host.querySelector('svg')!;
        p.cursor = p.host.querySelector<SVGLineElement>('.tp-cursor');
        p.readout = r.readout;
      });
    }
    parts.events.innerHTML = renderEvents(trace);
    parts.checks.innerHTML = renderChecks(trace);
    parts.prov.innerHTML = renderProvenance(trace, live);
    view?.setTrace(trace);
    seek(Math.min(index, n - 1));
  }

  // Keyboard, while focus is anywhere in the player. Native controls keep
  // their own keys (Space on a button, arrows on the slider).
  root.addEventListener('keydown', (ev) => {
    // Leave browser and OS shortcuts alone (Alt+← is back, Ctrl+Home, …).
    if (ev.ctrlKey || ev.metaKey || ev.altKey) return;
    const target = ev.target as HTMLElement;
    const native = target.matches('button, select, input');
    if (ev.key === ' ' || ev.key === 'k') {
      if (native && ev.key === ' ') return;
      setPlaying(!playing);
    } else if ((ev.key === 'ArrowRight' || ev.key === 'ArrowLeft') && !target.matches('input, select')) {
      setPlaying(false);
      seek(index + (ev.key === 'ArrowRight' ? 1 : -1) * (ev.shiftKey ? 10 : 1));
    } else if ((ev.key === 'Home' || ev.key === 'End') && !target.matches('input, select')) {
      setPlaying(false);
      seek(ev.key === 'Home' ? 0 : n - 1);
    } else if (ev.key === '[' || ev.key === ']') {
      const i = SPEEDS.indexOf(rate) + (ev.key === ']' ? 1 : -1);
      setRate(SPEEDS[Math.max(0, Math.min(SPEEDS.length - 1, i))]);
    } else {
      return;
    }
    ev.preventDefault();
  });
  root.tabIndex = 0;

  // The 3-D view loads three.js only when the player scrolls near view.
  const host = root.querySelector<HTMLElement>('[data-vehicle-view]');
  if (host) {
    const io = new IntersectionObserver(
      async (entries) => {
        if (!entries.some((e) => e.isIntersecting)) return;
        io.disconnect();
        try {
          const { createVehicleView } = await import('./vehicle-view');
          view = createVehicleView(host, trace as any, {
            thrustScale: config.thrustScale,
            reducedMotion: reducedMotion(),
            ghost: config.ghost,
          });
        } catch (err) {
          // A failed chunk load or a renderer error leaves the plots working.
          console.error('TracePlayer: 3-D view unavailable', err);
          view = null;
        }
        if (!view) {
          host.querySelector<HTMLElement>('[data-fallback]')!.hidden = false;
          return;
        }
        const buttons = [...host.querySelectorAll<HTMLButtonElement>('[data-camera]')];
        const select = (mode: string) => {
          buttons.forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.camera === mode)));
          view!.setCamera(mode);
        };
        buttons.forEach((b) => b.addEventListener('click', () => select(b.dataset.camera!)));
        // Under reduced motion the camera stays still: no chase camera.
        select(reducedMotion() ? 'overview' : 'chase');
        view.show(index);
      },
      { rootMargin: '200px' },
    );
    io.observe(host);
  }

  if (config.live) attachLive(root, config.live, recorded, load);

  setRate(1);
  render();
}

/** Moves a bar panel's bars, outlines, limit lines, and value labels to sample k. */
function showBars(svg: SVGSVGElement, trace: Trace, k: number, b: BarsReadout) {
  const row = (name?: string) => (name ? (trace.channels[name][k] as number[]) : undefined);
  const set = (el: Element | null, v: number) => {
    if (!el) return;
    const g = barGeometry(v, b.lo, b.hi);
    el.setAttribute('y', g.y.toFixed(1));
    el.setAttribute('height', g.height.toFixed(1));
  };
  const main = row(b.channel)!;
  const compare = row(b.compare);
  const marker = row(b.marker);
  const fmt = (v: number) => {
    const text = v.toFixed(b.decimals);
    return Number(text) === 0 ? (0).toFixed(b.decimals) : text;  // no "-0.0"
  };
  main.forEach((v, i) => {
    set(svg.querySelector(`[data-b="main-${i}"]`), v);
    if (compare) set(svg.querySelector(`[data-b="cmp-${i}"]`), compare[i]);
    if (marker) set(svg.querySelector(`[data-b="mk-${i}"]`), marker[i]);
    const label = svg.querySelector(`[data-b="val-${i}"]`);
    if (label) label.textContent = compare ? `${fmt(v)} | ${fmt(compare[i])}` : `${fmt(v)}${b.unit ? ` ${b.unit}` : ''}`;
  });
  const limits = row(b.limits);
  limits?.forEach((v, j) => {
    const line = svg.querySelector(`[data-b="lim-${j}"]`);
    const y = barGeometry(v, b.lo, b.hi).line.toFixed(1);
    line?.setAttribute('y1', y);
    line?.setAttribute('y2', y);
  });
}

/** The sliders of a live widget: each change reruns the scenario in WebAssembly. */
function attachLive(root: HTMLElement, live: LiveConfig, recorded: Trace, load: (trace: Trace, live: boolean) => void) {
  const box = root.querySelector<HTMLElement>('[data-live]')!;
  const sliders = [...box.querySelectorAll<HTMLInputElement>('[data-param]')];
  const outputs = [...box.querySelectorAll<HTMLOutputElement>('[data-param-value]')];
  const reset = box.querySelector<HTMLButtonElement>('[data-live-reset]')!;
  const status = box.querySelector<HTMLElement>('[data-live-status]')!;
  const display = (q: LiveParam) => Number((q.default / q.scale).toPrecision(6));
  let pending = false;

  const say = (text: string, error = false) => {
    status.textContent = text;
    status.classList.toggle('error', error);
  };
  const values = () => sliders.map((s, i) => Number(s.value) * live.params[i].scale);
  const atDefaults = () => sliders.every((s, i) => Number(s.value) === display(live.params[i]));
  const describe = () =>
    sliders.map((s, i) => `${live.params[i].label} = ${s.value} ${live.params[i].unit}`).join(', ');

  async function rerun() {
    pending = false;
    sliders.forEach((s, i) => (outputs[i].textContent = `${s.value} ${live.params[i].unit}`));
    if (atDefaults()) {
      load(recorded, false);
      reset.disabled = true;
      say('Showing the recorded run, at the default parameters.');
      return;
    }
    reset.disabled = false;
    say('Running…');
    let module;
    try {
      module = await loadModule(live.wasm);
    } catch (err) {
      console.warn('TracePlayer: WebAssembly module unavailable', err);
      say('Live runs are unavailable: this build of the site has no WebAssembly module, or it failed to load. The player shows the recorded run.', true);
      load(recorded, false);
      return;
    }
    try {
      const start = performance.now();
      const trace = runLive(module, live.scenario, values()) as Trace;
      const ms = performance.now() - start;
      load(trace, true);
      const edge = sliders.some((s) => s.value === s.min || s.value === s.max)
        ? ' This is the end of the range the scenario accepts.'
        : '';
      say(`Exploration, not evidence: computed in your browser in ${ms.toFixed(0)} ms, with ${describe()}. CI checks the recorded run, and the text below describes it.${edge}`);
    } catch (err) {
      console.error('TracePlayer: live run failed', err);
      say(`Could not run the scenario here: ${(err as Error).message}. The player shows the last run that succeeded.`, true);
    }
  }

  for (const s of sliders) {
    s.addEventListener('input', () => {
      if (pending) return;
      pending = true;
      requestAnimationFrame(rerun);
    });
  }
  reset.addEventListener('click', () => {
    sliders.forEach((s, i) => (s.value = String(display(live.params[i]))));
    rerun();
  });
}

export function initPlayers() {
  document.querySelectorAll<HTMLElement>('[data-trace-player]').forEach(attach);
}
