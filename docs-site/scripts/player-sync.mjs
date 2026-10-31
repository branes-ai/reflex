// player-sync.mjs — keeps node_modules/@branes-ai/trace-player in step with
// packages/trace-player. The site installs the package as a packed copy
// (.npmrc install-links), the files a consumer gets from the release tarball,
// and npm does not refresh that copy when the source changes. Run before the
// build, dev, and the trace scripts (the pre* scripts in package.json): when
// any packed file differs, it reinstalls the copy.

import { readFileSync, existsSync, rmSync, readdirSync, statSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { execFileSync } from 'node:child_process';

const site = resolve(import.meta.dirname, '..');
const source = resolve(site, '../packages/trace-player');
const installed = join(site, 'node_modules/@branes-ai/trace-player');

const walk = (dir) =>
  readdirSync(dir).flatMap((f) => {
    const p = join(dir, f);
    return statSync(p).isDirectory() ? walk(p) : [p];
  });
const manifest = JSON.parse(readFileSync(join(source, 'package.json'), 'utf8'));
const packed = ['package.json', ...manifest.files]
  .map((f) => join(source, f))
  .filter(existsSync)
  .flatMap((p) => (statSync(p).isDirectory() ? walk(p) : [p]))
  .map((p) => relative(source, p));

const copies = existsSync(installed)
  ? walk(installed).map((p) => relative(installed, p)).filter((f) => !f.startsWith('node_modules'))
  : [];
const stale =
  packed.find((f) => {
    const copy = join(installed, f);
    return !existsSync(copy) || !readFileSync(copy).equals(readFileSync(join(source, f)));
  }) ?? copies.find((f) => !packed.includes(f) && !/^(README|LICENSE)/i.test(f));
if (stale) {
  console.log(`player-sync: ${stale} changed in packages/trace-player; reinstalling the package`);
  rmSync(installed, { recursive: true, force: true });
  execFileSync('npm', ['install', '--no-audit', '--no-fund'], { cwd: site, stdio: 'inherit' });
}
