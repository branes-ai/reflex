// wasm-agree.mjs — runs every live scenario in the WebAssembly module at its
// default parameters and compares the result with the committed native trace
// under the `wasm` equivalence profile (Tier 2 of docs/standards/animation).
//
//   node scripts/trace/wasm-agree.mjs [path/to/reflex.mjs]   (default: public/wasm/reflex.mjs)
//
// Fails if the module is missing, a scenario fails or returns an invalid
// trace, or a trace differs from its native counterpart.

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { LIVE, runLive } from '../../src/lib/trace/live.js';
import { validateTrace } from '../../src/lib/trace/validate.js';
import { diffTraces } from '../../src/lib/trace/diff.js';

const here = dirname(fileURLToPath(import.meta.url));
const modulePath = resolve(process.argv[2] ?? resolve(here, '../../public/wasm/reflex.mjs'));
const { default: createReflex } = await import(pathToFileURL(modulePath).href);
const module = await createReflex();
console.log(`module ${modulePath} (built at ${module.ccall('reflex_version', 'string', [], [])})`);

let failed = false;
for (const [name, spec] of Object.entries(LIVE)) {
  const native = JSON.parse(readFileSync(resolve(here, '../../src/traces', spec.trace), 'utf8'));
  let wasm;
  try {
    wasm = runLive(module, name, spec.params.map((p) => p.default));
  } catch (err) {
    console.error(`FAIL ${name}: ${err.message}`);
    failed = true;
    continue;
  }
  const errors = validateTrace(wasm);
  const { equivalent, failures } = diffTraces(wasm, native, 'wasm');
  if (errors.length || !equivalent) {
    console.error(`FAIL ${name} (${wasm.provenance.compiler}) against ${spec.trace}`);
    for (const e of [...errors, ...failures]) console.error(`  ${e}`);
    failed = true;
  } else {
    console.log(`ok   ${name}: ${wasm.provenance.compiler} agrees with ${spec.trace} (${native.provenance.compiler}) under wasm`);
  }
}
process.exit(failed ? 1 : 0);
