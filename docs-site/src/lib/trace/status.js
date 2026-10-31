// A check's outcome against what its scenario expects of it. A check passes
// unless the trace marks it `"expect": "fail"`: a scenario that shows the edge
// of a block's envelope. Used by the player's check list, the scenario
// gallery, and scripts/trace/check.mjs, which fails CI on a failing scenario.

/** Labels, for the badges and the gallery. */
export const STATUS_LABEL = {
  pass: 'pass',
  designed: 'fails, as designed',
  fail: 'fail',
  unexpected: 'passes, but is meant to fail',
  none: 'no checks',
};

/** 'pass', 'designed' (fails, as expected), 'fail', or 'unexpected' (passes, expected to fail). */
export function checkStatus(check) {
  const expectFail = check.expect === 'fail';
  if (check.pass) return expectFail ? 'unexpected' : 'pass';
  return expectFail ? 'designed' : 'fail';
}

/**
 * A trace's outcome: 'fail' when any check fails or passes against its
 * expectation, else 'designed' when one fails as designed, else 'pass';
 * 'none' for a trace without checks.
 */
export function traceStatus(trace) {
  const all = (trace.checks ?? []).map(checkStatus);
  if (all.length === 0) return 'none';
  if (all.some((s) => s === 'fail' || s === 'unexpected')) return 'fail';
  return all.includes('designed') ? 'designed' : 'pass';
}

/** Whether a status counts as failing. */
export const failing = (status) => status === 'fail' || status === 'unexpected';
