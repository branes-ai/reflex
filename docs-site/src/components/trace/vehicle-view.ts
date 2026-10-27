// VehicleView: the three.js scene of TracePlayer. World axes and a ground grid
// (z up), a PX4 Quad X glyph with rotor indices and spin arrows, per-rotor
// thrust arrows to a stated scale, body axes, and the trail so far. An
// optional ghost draws a second recorded run (for example a mirrored one)
// beside the first, translucent. It shows recorded samples only; it renders
// on demand, not in a loop.

import * as THREE from 'three';
import type { ViewHandle } from './player';

interface Trace {
  t: number[];
  channels: Record<string, (number | number[])[]>;
  vehicle?: { arm_length?: number };
}

const DEFAULT_ARM = 0.11;
// PX4 Quad X, model order: 0 FR, 1 RL, 2 FL, 3 RR, as (x, y) signs × arm/√2.
const ROTOR_SIGNS: [number, number][] = [[1, -1], [-1, 1], [1, 1], [-1, -1]];
const ROTOR_CCW = [true, true, false, false];

function tokens(el: HTMLElement) {
  const cs = getComputedStyle(el);
  const v = (name: string) => new THREE.Color(cs.getPropertyValue(name).trim() || '#888');
  return {
    ink: v('--tp-ink'), ink2: v('--tp-ink2'), grid: v('--tp-grid'), axis: v('--tp-axis'),
    series: [v('--tp-s1'), v('--tp-s2'), v('--tp-s3'), v('--tp-s4')],
    x: v('--tp-x'), y: v('--tp-y'), z: v('--tp-z'),
  };
}

function textSprite(text: string, color: THREE.Color, size: number) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 64;
  const ctx = canvas.getContext('2d')!;
  ctx.font = '600 44px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = `#${color.getHexString()}`;
  ctx.fillText(text, 32, 34);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, depthTest: false, transparent: true }));
  sprite.scale.set(size, size, 1);
  sprite.renderOrder = 10;
  return sprite;
}

function axes(length: number, c: ReturnType<typeof tokens>, labels: boolean) {
  const group = new THREE.Group();
  const dirs = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
  [c.x, c.y, c.z].forEach((color, i) => {
    group.add(new THREE.ArrowHelper(dirs[i], new THREE.Vector3(), length, color, length * 0.18, length * 0.09));
    if (labels) {
      const s = textSprite('xyz'[i], color, length * 0.35);
      s.position.copy(dirs[i]).multiplyScalar(length * 1.18);
      group.add(s);
    }
  });
  return group;
}

/** An arc arrow around +z showing the rotor's spin direction. */
function spinArrow(radius: number, ccw: boolean, color: THREE.Color) {
  const group = new THREE.Group();
  // Double-sided: the clockwise arrow is a mirrored copy (negative scale).
  const material = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
  const arc = Math.PI * 1.4;
  const tube = new THREE.Mesh(new THREE.TorusGeometry(radius, radius * 0.05, 6, 32, arc), material);
  group.add(tube);
  const head = new THREE.Mesh(new THREE.ConeGeometry(radius * 0.16, radius * 0.4, 12), material);
  // The torus arc runs CCW (seen from +z) from angle 0 to `arc`; put the head
  // at the end, pointing along the tangent.
  head.position.set(radius * Math.cos(arc), radius * Math.sin(arc), 0);
  head.rotation.z = arc;  // cone axis (+y) → tangent of a CCW arc
  group.add(head);
  if (!ccw) group.scale.y = -1;  // mirror: the same arc, clockwise
  return group;
}

/** Channels of a second run drawn as a translucent ghost. */
export interface GhostSpec {
  position: string;
  attitude: string;
  thrusts?: string;
  label: string;
}

/** The Quad X glyph in the body frame, with its thrust arrows. */
function glyph(arm: number, c: ReturnType<typeof tokens>, opacity: number) {
  const a = arm / Math.SQRT2;
  const body = new THREE.Group();
  const arrows: THREE.ArrowHelper[] = [];
  const material = (color: THREE.Color, extra: THREE.MeshBasicMaterialParameters = {}) =>
    new THREE.MeshBasicMaterial({ color, ...extra });
  const armMat = material(c.ink2);
  for (const [i, j] of [[0, 1], [2, 3]]) {
    const p = new THREE.Vector3(ROTOR_SIGNS[i][0] * a, ROTOR_SIGNS[i][1] * a, 0);
    const q = new THREE.Vector3(ROTOR_SIGNS[j][0] * a, ROTOR_SIGNS[j][1] * a, 0);
    const beam = new THREE.Mesh(new THREE.BoxGeometry(p.distanceTo(q), arm * 0.06, arm * 0.04), armMat);
    beam.position.copy(p).add(q).multiplyScalar(0.5);
    beam.rotation.z = Math.atan2(q.y - p.y, q.x - p.x);
    body.add(beam);
  }
  // Nose marker on +x: the forward direction.
  const nose = new THREE.Mesh(new THREE.ConeGeometry(arm * 0.12, arm * 0.35, 12), material(c.x));
  nose.rotation.z = -Math.PI / 2;
  nose.position.set(a * 0.9, 0, 0);
  body.add(nose);
  ROTOR_SIGNS.forEach(([sx, sy], i) => {
    const r = new THREE.Vector3(sx * a, sy * a, 0);
    const color = c.series[i];
    const disc = new THREE.Mesh(new THREE.RingGeometry(arm * 0.36, arm * 0.42, 40), material(color, { side: THREE.DoubleSide }));
    disc.position.copy(r);
    body.add(disc);
    const spin = spinArrow(arm * 0.26, ROTOR_CCW[i], color);
    spin.position.copy(r).add(new THREE.Vector3(0, 0, arm * 0.02));
    body.add(spin);
    const label = textSprite(String(i), c.ink, arm * 0.5);
    label.position.copy(r).add(new THREE.Vector3(0, 0, -arm * 0.3));
    body.add(label);
    const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), r, 0.1, color, arm * 0.2, arm * 0.12);
    arrows.push(arrow);
    body.add(arrow);
  });
  body.add(axes(arm * 1.4, c, false));
  // A ghost: every material of the glyph, sprites and arrows included.
  if (opacity < 1) {
    body.traverse((o) => {
      const m = (o as THREE.Mesh).material as THREE.Material | undefined;
      if (m) {
        m.transparent = true;
        m.opacity = opacity;
      }
    });
  }
  return { body, arrows };
}

export function createVehicleView(
  host: HTMLElement,
  trace: Trace,
  opts: { thrustScale: number; reducedMotion: boolean; ghost?: GhostSpec },
): ViewHandle | null {
  const mount = host.querySelector<HTMLElement>('[data-canvas]')!;
  let renderer: THREE.WebGLRenderer;
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  } catch {
    return null;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  mount.appendChild(renderer.domElement);
  renderer.domElement.setAttribute('aria-hidden', 'true');

  const pos = trace.channels.position as number[][];
  const att = trace.channels.attitude as number[][];
  const thrusts = trace.channels.rotor_thrusts as number[][] | undefined;
  const ghost = opts.ghost
    ? {
        pos: trace.channels[opts.ghost.position] as number[][],
        att: trace.channels[opts.ghost.attitude] as number[][],
        thrusts: opts.ghost.thrusts ? (trace.channels[opts.ghost.thrusts] as number[][]) : undefined,
      }
    : null;
  const arm = trace.vehicle?.arm_length ?? DEFAULT_ARM;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 1000);
  camera.up.set(0, 0, 1);

  // Bounds of the trajectories, for the grid and the overview camera.
  const box = new THREE.Box3();
  for (const path of ghost ? [pos, ghost.pos] : [pos]) {
    path.forEach((p) => box.expandByPoint(new THREE.Vector3(p[0], p[1], p[2])));
  }
  box.expandByPoint(new THREE.Vector3(box.min.x, box.min.y, 0));
  const center = box.getCenter(new THREE.Vector3());
  const extentXY = Math.max(box.max.x - box.min.x, box.max.y - box.min.y, 1);
  const extent = Math.max(extentXY, box.max.z - box.min.z);

  let built: THREE.Object3D[] = [];
  let main: ReturnType<typeof glyph>;
  let shadow: ReturnType<typeof glyph> | null = null;
  let trail: THREE.Line;
  let ghostTrail: THREE.Line | null = null;
  let cameraMode = 'chase';
  let index = 0;

  const line = (path: number[][], color: THREE.Color, opacity: number) =>
    new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(path.map((p) => new THREE.Vector3(p[0], p[1], p[2]))),
      new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }),
    );

  function build() {
    built.forEach((o) => scene.remove(o));
    const c = tokens(host);

    // Ground grid on z = 0, 0.5 m squares, covering the trajectory.
    const size = Math.ceil(extentXY + 2);
    const grid = new THREE.GridHelper(size, size * 2, c.axis, c.grid);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(Math.round(center.x * 2) / 2, Math.round(center.y * 2) / 2, 0);
    const world = axes(0.3, c, true);

    main = glyph(arm, c, 1);
    // Trail: the whole path, drawn up to the current sample.
    trail = line(pos, c.ink2, 1);
    built = [grid, world, main.body, trail];
    if (ghost) {
      shadow = glyph(arm, c, 0.4);
      ghostTrail = line(ghost.pos, c.ink2, 0.4);
      built.push(shadow.body, ghostTrail);
    }
    built.forEach((o) => scene.add(o));
  }

  function resize() {
    const w = mount.clientWidth;
    const h = mount.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  function place(g: ReturnType<typeof glyph>, p: number[], q: number[], f: number[] | undefined) {
    g.body.position.set(p[0], p[1], p[2]);
    g.body.quaternion.set(q[1], q[2], q[3], q[0]);  // trace is [w, x, y, z]; three is (x, y, z, w)
    g.arrows.forEach((arrow, i) => {
      const thrust = f ? f[i] : 0;
      arrow.visible = thrust > 0;
      if (thrust > 0) arrow.setLength(thrust * opts.thrustScale, arm * 0.2, arm * 0.12);
    });
  }

  function draw() {
    const p = pos[index];
    place(main, p, att[index], thrusts?.[index]);
    trail.geometry.setDrawRange(0, index + 1);
    const target = new THREE.Vector3(p[0], p[1], p[2]);
    let reach = 1;
    if (ghost && shadow && ghostTrail) {
      const g = ghost.pos[index];
      place(shadow, g, ghost.att[index], ghost.thrusts?.[index]);
      ghostTrail.geometry.setDrawRange(0, index + 1);
      // Follow the midpoint, far enough back to keep both vehicles in view.
      const other = new THREE.Vector3(g[0], g[1], g[2]);
      reach = Math.max(1, 1.2 * target.distanceTo(other));
      target.add(other).multiplyScalar(0.5);
    }

    if (cameraMode === 'chase') {
      camera.position.copy(target).add(new THREE.Vector3(-0.55, -0.75, 0.45).multiplyScalar(reach));
      camera.lookAt(target);
    } else if (cameraMode === 'behind') {
      camera.position.copy(target).add(new THREE.Vector3(-0.95, 0, 0.18).multiplyScalar(reach));
      camera.lookAt(target);
    } else {
      const d = extent * 1.1 + 1;
      camera.position.set(center.x - d * 0.6, center.y - d * 0.9, center.z + d * 0.6);
      camera.lookAt(center);
    }
    renderer.render(scene, camera);
  }

  build();
  resize();
  new ResizeObserver(() => {
    resize();
    draw();
  }).observe(mount);
  // Rebuild the materials when the docs theme or the OS scheme changes.
  const rebuild = () => {
    build();
    draw();
  };
  new MutationObserver(rebuild).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', rebuild);

  return {
    show(k: number) {
      index = k;
      draw();
    },
    setCamera(mode: string) {
      cameraMode = mode;
      draw();
    },
  };
}
