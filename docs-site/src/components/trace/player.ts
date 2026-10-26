// Client side of TracePlayer: the time base, controls, keyboard, plot cursors
// and readouts, and the lazily loaded three.js vehicle view.
//
// Rules from the animation standard (docs/standards/animation):
//   - starts paused, and only a reader action plays it (ANI-10);
//   - the rate is always on screen (ANI-5);
//   - sample-and-hold: the cursor sits on a recorded sample, never between
//     samples, so nothing on screen is computed here (ANI-1).

interface Readout {
  channel: string;
  comps: number[];
  decimals: number;
}

interface Payload {
  trace: {
    t: number[];
    channels: Record<string, (number | number[])[]>;
    events?: { t: number; label: string }[];
    vehicle?: { arm_length?: number };
  };
  config: { thrustScale: number; panels: Readout[] };
}

export interface ViewHandle {
  show(index: number): void;
  setCamera(mode: string): void;
}

const reducedMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const SPEEDS = [0.1, 0.25, 0.5, 1];

function attach(root: HTMLElement) {
  const payload: Payload = JSON.parse(root.querySelector('script[data-trace]')!.textContent!);
  const { trace, config } = payload;
  const t = trace.t;
  const n = t.length;
  const play = root.querySelector<HTMLButtonElement>('[data-play]')!;
  const icon = root.querySelector<SVGPathElement>('[data-play-icon]')!;
  const scrub = root.querySelector<HTMLInputElement>('[data-scrub]')!;
  const time = root.querySelector<HTMLElement>('[data-time]')!;
  const speed = root.querySelector<HTMLSelectElement>('[data-speed]')!;
  const badge = root.querySelector<HTMLElement>('[data-rate-badge]')!;
  const eventBox = root.querySelector<HTMLElement>('[data-event]');
  const panels = [...root.querySelectorAll<HTMLElement>('[data-panel]')].map((el, i) => ({
    svg: el.querySelector<SVGSVGElement>('svg')!,
    cursor: el.querySelector<SVGLineElement>('.tp-cursor')!,
    values: [...el.querySelectorAll<HTMLElement>('[data-readout]')],
    readout: config.panels[i],
  }));

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
      const x0 = Number(p.svg.dataset.x0);
      const x1 = Number(p.svg.dataset.x1);
      const x = x0 + ((tk - t[0]) / (t[n - 1] - t[0])) * (x1 - x0);
      p.cursor.setAttribute('x1', String(x));
      p.cursor.setAttribute('x2', String(x));
      const entry = trace.channels[p.readout.channel][index];
      p.readout.comps.forEach((c, j) => {
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
  root.querySelectorAll<HTMLButtonElement>('[data-seek]').forEach((b) =>
    b.addEventListener('click', () => {
      setPlaying(false);
      seek(indexAt(Number(b.dataset.seek)));
    }),
  );

  // Scrubbing on any plot.
  for (const p of panels) {
    const toIndex = (ev: PointerEvent) => {
      const ctm = p.svg.getScreenCTM();
      if (!ctm) return index;
      const x = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(ctm.inverse()).x;
      const x0 = Number(p.svg.dataset.x0);
      const x1 = Number(p.svg.dataset.x1);
      return indexAt(t[0] + ((x - x0) / (x1 - x0)) * (t[n - 1] - t[0]));
    };
    p.svg.addEventListener('pointerdown', (ev) => {
      setPlaying(false);
      p.svg.setPointerCapture(ev.pointerId);
      seek(toIndex(ev));
    });
    p.svg.addEventListener('pointermove', (ev) => {
      if (p.svg.hasPointerCapture(ev.pointerId)) seek(toIndex(ev));
    });
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
          view = createVehicleView(host, trace, { thrustScale: config.thrustScale, reducedMotion: reducedMotion() });
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

  setRate(1);
  render();
}

export function initPlayers() {
  document.querySelectorAll<HTMLElement>('[data-trace-player]').forEach(attach);
}
