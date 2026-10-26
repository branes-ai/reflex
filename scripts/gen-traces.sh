#!/usr/bin/env bash
# gen-traces.sh — build and run every trace scenario (tools/trace/scenarios/),
# writing reflex-trace/1 files under <out-dir>/<block>/<scenario>.json.
#
#   scripts/gen-traces.sh <preset> <out-dir>
#
# Regenerate the committed docs traces after a model change:
#   scripts/gen-traces.sh gcc-release docs-site/src/traces
# CI runs it into build/traces and compares the result with the committed
# copies (docs-site/scripts/trace/check.mjs --regen).
set -euo pipefail

preset=${1:?usage: gen-traces.sh <preset> <out-dir>}
out=${2:?usage: gen-traces.sh <preset> <out-dir>}
mkdir -p "$out"
out=$(cd "$out" && pwd)  # relative to the caller's directory
root=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
cd "$root"

cmake --build --preset "$preset" --target reflex_traces
bin="build/$preset/tools/trace"

# The commit the traces were generated at; "-dirty" marks uncommitted changes.
commit=$(git describe --always --dirty --abbrev=7 --exclude '*' 2>/dev/null || echo unknown)

shopt -s nullglob
scenarios=("$bin"/trace_*)
if [ ${#scenarios[@]} -eq 0 ]; then
    echo "no trace scenarios in $bin" >&2
    exit 1
fi
for exe in "${scenarios[@]}"; do
    [ -x "$exe" ] && [ -f "$exe" ] || continue
    REFLEX_TRACE_DIR="$out" REFLEX_TRACE_COMMIT="$commit" "$exe"
done
