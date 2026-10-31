// trace-diff.mjs — trace_diff (BB-D3): compares a candidate trace with a
// reference under an equivalence profile and reports the first sample and
// channel that fail.
//
//   node scripts/trace/trace-diff.mjs <candidate.json> <reference.json> [--profile regen|wasm]

import { readFileSync } from 'node:fs';
import { diffTraces } from '@branes-ai/trace-player/diff';
import { validateTrace } from '@branes-ai/trace-player/validate';

const args = process.argv.slice(2);
const p = args.indexOf('--profile');
const profile = p >= 0 ? args[p + 1] : 'regen';
const files = p >= 0 ? args.filter((_, i) => i !== p && i !== p + 1) : args;
if (files.length !== 2) {
  console.error('usage: trace-diff.mjs <candidate.json> <reference.json> [--profile regen|wasm]');
  process.exit(2);
}
const [a, b] = files.map((f) => JSON.parse(readFileSync(f, 'utf8')));
for (const [f, tr] of [[files[0], a], [files[1], b]]) {
  const errors = validateTrace(tr);
  if (errors.length) {
    console.error(`${f}: invalid trace`);
    for (const e of errors) console.error(`  ${e}`);
    process.exit(1);
  }
}
const { equivalent, failures } = diffTraces(a, b, profile);
if (equivalent) {
  console.log(`equivalent under ${profile}`);
} else {
  console.log(`not equivalent under ${profile}:`);
  for (const f of failures) console.log(`  ${f}`);
}
process.exit(equivalent ? 0 : 1);
