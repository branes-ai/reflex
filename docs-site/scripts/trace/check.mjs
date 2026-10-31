// check.mjs — validates every committed reflex-trace/1 file, and with --regen
// compares each against a freshly regenerated copy under the `regen`
// equivalence profile (ANI-3).
//
//   node scripts/trace/check.mjs                       # validate src/traces/**
//   node scripts/trace/check.mjs --regen ../build/traces
//   node scripts/trace/check.mjs --regen ../build/traces-gcc --report src/run/report.json --label gcc
//
// A committed trace with no regenerated counterpart, or a regenerated trace
// that is not committed, fails: every trace on the site comes from a scenario
// that CI runs. So does a failing scenario: a check that fails, or that
// passes when the trace marks it as expected to fail (src/lib/trace/status.js).
//
// --report adds this run's outcome per trace, under --label, to a JSON file
// that the scenario gallery shows (the deploy workflow writes it, nightly):
//   { "commit", "date", "run", "runs": { "<label>": { "compiler",
//     "traces": { "<block>/<name>.json": "match" | "stale" | "missing" | "invalid" },
//     "extra": [ "<block>/<name>.json", … ] } }, "published": "<label>" }
//
// --publish DIR copies the regenerated traces that are valid into DIR (the
// deploy workflow passes src/traces for the gcc run), so that the site plays
// the fresh run; where the run wrote an invalid trace, or none, the committed
// one stays. The report names that run as `published`.

import { readFileSync, readdirSync, existsSync, writeFileSync, mkdirSync, copyFileSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { execFileSync } from 'node:child_process';
import { validateTrace } from '../../src/lib/trace/validate.js';
import { diffTraces } from '../../src/lib/trace/diff.js';
import { traceStatus, failing, STATUS_LABEL } from '../../src/lib/trace/status.js';

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
const option = (name) => (args.includes(name) ? args[args.indexOf(name) + 1] : undefined);
const reportPath = option('--report');
const label = option('--label') ?? 'regen';
const publishDir = option('--publish');
if ((reportPath || publishDir) && !regenDir) {
  console.error('--report and --publish need --regen');
  process.exit(2);
}
const outcome = {};
const valid = [];  // regenerated traces that parse and validate, for --publish
let compiler;

let failed = false;
const fail = (msg) => {
  console.error(`FAIL ${msg}`);
  failed = true;
};

/** The regenerated trace, or undefined (and a failure) when it is not valid. */
function loadRegen(rel) {
  let trace;
  try {
    trace = JSON.parse(readFileSync(join(regenDir, rel), 'utf8'));
  } catch (e) {
    fail(`${rel}: regenerated trace is not JSON: ${e.message}`);
    return undefined;
  }
  const errors = validateTrace(trace);
  if (errors.length) {
    fail(`${rel}: regenerated trace is invalid`);
    for (const e of errors) console.error(`  ${e}`);
    return undefined;
  }
  return trace;
}

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
  const status = traceStatus(trace);
  if (failing(status)) {
    fail(`${rel}: failing scenario`);
    for (const c of trace.checks) console.error(`  ${c.id} ${c.metric} = ${c.value} (${c.comparison} ${c.limit}): ${c.pass ? 'pass' : 'fail'}${c.expect === 'fail' ? ', expected to fail' : ''}`);
  }
  if (!regenDir) {
    if (!failing(status)) console.log(`ok   ${rel} (valid, ${STATUS_LABEL[status]})`);
    continue;
  }
  const regenPath = join(regenDir, rel);
  if (!existsSync(regenPath)) {
    fail(`${rel}: no regenerated trace at ${regenPath}`);
    outcome[rel] = 'missing';
    continue;
  }
  const regen = loadRegen(rel);
  if (!regen) {
    outcome[rel] = 'invalid';
    continue;
  }
  valid.push(rel);
  compiler ??= regen.provenance.compiler;
  const { equivalent, failures } = diffTraces(regen, trace, 'regen');
  if (!equivalent) {
    fail(`${rel}: stale — the regenerated trace differs from the committed one`);
    for (const f of failures) console.error(`  ${f}`);
    console.error('  regenerate with scripts/gen-traces.sh and commit the result');
    outcome[rel] = 'stale';
    continue;
  }
  outcome[rel] = 'match';
  if (!failing(status)) console.log(`ok   ${rel} (valid, ${STATUS_LABEL[status]}, matches regenerated under regen)`);
}

const extra = [];
if (regenDir) {
  for (const rel of listJson(regenDir)) {
    if (!committed.includes(rel)) {
      fail(`${rel}: regenerated but not committed under src/traces/`);
      extra.push(rel);
      if (loadRegen(rel)) valid.push(rel);
    }
  }
}

if (reportPath) {
  const path = resolve(reportPath);
  const report = existsSync(path) ? JSON.parse(readFileSync(path, 'utf8')) : { runs: {} };
  const env = process.env;
  const head = () => {
    try {
      return execFileSync('git', ['rev-parse', '--short=7', 'HEAD'], { encoding: 'utf8' }).trim();
    } catch {
      return 'unknown';
    }
  };
  report.commit = env.GITHUB_SHA?.slice(0, 7) ?? report.commit ?? head();
  report.date ??= new Date().toISOString();
  if (env.GITHUB_RUN_ID) report.run = `${env.GITHUB_SERVER_URL}/${env.GITHUB_REPOSITORY}/actions/runs/${env.GITHUB_RUN_ID}`;
  report.runs[label] = { compiler: compiler ?? 'unknown', traces: outcome, extra };
  if (publishDir) report.published = label;
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, `${JSON.stringify(report, null, 2)}\n`);
  console.log(`wrote ${relative(process.cwd(), path)} (${label})`);
}

if (publishDir) {
  for (const rel of valid) {
    const to = join(resolve(publishDir), rel);
    mkdirSync(dirname(to), { recursive: true });
    copyFileSync(join(regenDir, rel), to);
  }
  console.log(`copied ${valid.length} regenerated traces to ${relative(process.cwd(), resolve(publishDir))}`);
}

process.exit(failed ? 1 : 0);
