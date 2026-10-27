// size-budget.mjs — ANI-12. Lists every counted file with its gzip -9 size and
// fails over budget:
//   - each committed trace (src/traces/**.json): ≤ 250 KB;
//   - each WebAssembly module plus its JavaScript loader, as built into
//     dist/wasm/ (scripts/build-wasm.sh, copied from public/wasm/): ≤ 400 KB;
//   - three.js is outside the budget, but may appear only in the built bundles
//     of pages that use the 3-D view (a [data-vehicle-view] element).
//
// Run after `astro build` (it reads dist/). With --require-wasm, a build
// without a WebAssembly module fails too (CI builds one).

import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { dirname, join, relative, resolve, basename } from 'node:path';
import { fileURLToPath } from 'node:url';
import { gzipSync } from 'node:zlib';

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, '../..');
const KB = 1000;
let failed = false;

const gz = (path) => gzipSync(readFileSync(path), { level: 9 }).length;
const list = (dir, ext) =>
  existsSync(dir)
    ? readdirSync(dir, { recursive: true }).filter((f) => ext.some((e) => f.endsWith(e))).map((f) => join(dir, f)).sort()
    : [];
const row = (label, size, budget) => {
  const over = size > budget;
  if (over) failed = true;
  console.log(`${over ? 'FAIL' : 'ok  '} ${(size / KB).toFixed(1).padStart(7)} KB / ${budget / KB} KB  ${label}`);
};

console.log('Traces (gzip -9 each, budget 250 KB):');
for (const f of list(join(root, 'src/traces'), ['.json'])) row(relative(root, f), gz(f), 250 * KB);

console.log('WebAssembly module + loader (gzip -9 together, budget 400 KB):');
const wasm = list(join(root, 'dist/wasm'), ['.wasm']);
if (wasm.length === 0) {
  if (process.argv.includes('--require-wasm')) {
    failed = true;
    console.log('FAIL no module in dist/wasm/: run scripts/build-wasm.sh before the site build');
  } else {
    console.log('     (none built: scripts/build-wasm.sh)');
  }
}
for (const f of wasm) {
  const loader = f.replace(/\.wasm$/, '.mjs');
  if (!existsSync(loader)) {
    failed = true;
    console.log(`FAIL ${relative(root, f)} has no loader ${basename(loader)}`);
    continue;
  }
  row(`${relative(root, f)} + ${basename(loader)}`, gz(f) + gz(loader), 400 * KB);
}

// three.js placement: follow each page's module scripts through their static
// and dynamic imports, and flag any page without a 3-D view that reaches a
// chunk containing three.js.
const dist = join(root, 'dist');
if (!existsSync(dist)) {
  console.error('FAIL dist/ not found: run `npm run build` first');
  process.exit(1);
}
const chunkCache = new Map();
function chunkInfo(path) {
  if (!chunkCache.has(path)) {
    const src = existsSync(path) ? readFileSync(path, 'utf8') : '';
    const imports = [...src.matchAll(/(?:import|from)\s*\(?\s*["'](\.{1,2}\/[^"']+\.js)["']/g)].map((m) => resolve(dirname(path), m[1]));
    // three.js stamps its revision into the bundle.
    chunkCache.set(path, { three: /REVISION\s*=\s*["']\d+/.test(src) || src.includes('__THREE__'), imports });
  }
  return chunkCache.get(path);
}
function reachesThree(entry, seen = new Set()) {
  if (seen.has(entry)) return false;
  seen.add(entry);
  const info = chunkInfo(entry);
  return info.three || info.imports.some((i) => reachesThree(i, seen));
}
console.log('three.js placement (only pages with a 3-D view):');
let threePages = 0;
for (const page of list(dist, ['.html'])) {
  const html = readFileSync(page, 'utf8');
  const scripts = [...html.matchAll(/<script[^>]*type="module"[^>]*src="([^"]+)"/g)].map((m) => m[1]);
  const inline = [...html.matchAll(/<script[^>]*type="module"[^>]*>([\s\S]*?)<\/script>/g)].flatMap((m) =>
    [...m[1].matchAll(/["'](\/[^"']+\.js)["']/g)].map((x) => x[1]),
  );
  const entries = [...scripts, ...inline].map((s) => join(dist, s.replace(/^\/reflex\//, '/')));
  const usesThree = entries.some((e) => reachesThree(e));
  const hasView = html.includes('data-vehicle-view');
  if (usesThree) threePages++;
  if (usesThree && !hasView) {
    failed = true;
    console.log(`FAIL ${relative(dist, page)} loads three.js without a 3-D view`);
  } else if (hasView && !usesThree) {
    failed = true;
    console.log(`FAIL ${relative(dist, page)} has a 3-D view but no three.js bundle was found (check the detector)`);
  }
}
console.log(`     ${threePages} page(s) load three.js, each with a 3-D view`);

process.exit(failed ? 1 : 0);
