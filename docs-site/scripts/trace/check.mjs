// check.mjs — validates every committed reflex-trace/1 file, and with --regen
// compares each against a freshly regenerated copy under the `regen`
// equivalence profile (ANI-3).
//
//   node scripts/trace/check.mjs                       # validate src/traces/**
//   node scripts/trace/check.mjs --regen ../build/traces
//
// A committed trace with no regenerated counterpart, or a regenerated trace
// that is not committed, fails: every trace on the site comes from a scenario
// that CI runs.

import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { validateTrace } from '../../src/lib/trace/validate.js';
import { diffTraces } from '../../src/lib/trace/diff.js';

const here = dirname(fileURLToPath(import.meta.url));
const committedDir = resolve(here, '../../src/traces');

function listJson(dir) {
  if (!existsSync(dir)) return [];
  return readdirSync(dir, { recursive: true })
    .filter((f) => f.endsWith('.json'))
    .map((f) => f.split('\\').join('/'))
    .sort();
}

const args = process.argv.slice(2);
const regenIndex = args.indexOf('--regen');
const regenDir = regenIndex >= 0 ? resolve(args[regenIndex + 1] ?? '') : null;

let failed = false;
const fail = (msg) => {
  console.error(`FAIL ${msg}`);
  failed = true;
};

const committed = listJson(committedDir);
if (committed.length === 0) fail(`no traces under ${relative(process.cwd(), committedDir)}`);

for (const rel of committed) {
  const trace = JSON.parse(readFileSync(join(committedDir, rel), 'utf8'));
  const errors = validateTrace(trace);
  if (errors.length) {
    fail(`${rel}: invalid`);
    for (const e of errors) console.error(`  ${e}`);
    continue;
  }
  if (!regenDir) {
    console.log(`ok   ${rel} (valid)`);
    continue;
  }
  const regenPath = join(regenDir, rel);
  if (!existsSync(regenPath)) {
    fail(`${rel}: no regenerated trace at ${regenPath}`);
    continue;
  }
  const regen = JSON.parse(readFileSync(regenPath, 'utf8'));
  const regenErrors = validateTrace(regen);
  if (regenErrors.length) {
    fail(`${rel}: regenerated trace is invalid`);
    for (const e of regenErrors) console.error(`  ${e}`);
    continue;
  }
  const { equivalent, failures } = diffTraces(regen, trace, 'regen');
  if (!equivalent) {
    fail(`${rel}: stale — the regenerated trace differs from the committed one`);
    for (const f of failures) console.error(`  ${f}`);
    console.error('  regenerate with scripts/gen-traces.sh and commit the result');
    continue;
  }
  console.log(`ok   ${rel} (valid, matches regenerated under regen)`);
}

if (regenDir) {
  for (const rel of listJson(regenDir)) {
    if (!committed.includes(rel)) fail(`${rel}: regenerated but not committed under src/traces/`);
  }
}

process.exit(failed ? 1 : 0);
