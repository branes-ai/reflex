#!/usr/bin/env bash
# build-wasm.sh — build the docs-site's live-widget module with Emscripten and
# copy it into the site.
#
#   scripts/build-wasm.sh [out-dir]      (default: docs-site/public/wasm)
#
# Needs the pinned emsdk on PATH (source emsdk_env.sh); CI pins it in
# .github/workflows/ci.yml and deploy-docs.yml. Writes reflex.mjs and
# reflex.wasm; the site loads them only when a reader moves a widget slider.
set -euo pipefail

root=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
out=${1:-$root/docs-site/public/wasm}
mkdir -p "$out"
out=$(cd "$out" && pwd)
cd "$root"

command -v emcmake >/dev/null || { echo "emcmake not found: source emsdk_env.sh first" >&2; exit 1; }

commit=$(git describe --always --dirty --abbrev=7 --exclude '*' 2>/dev/null || echo unknown)
emcmake cmake -S . -B build/wasm -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DREFLEX_BUILD_TESTS=OFF -DREFLEX_BUILD_BENCH=OFF -DREFLEX_BUILD_TOOLS=ON \
    -DREFLEX_WASM_COMMIT="$commit" >/dev/null
cmake --build build/wasm --target reflex_wasm
cp build/wasm/wasm/reflex.mjs build/wasm/wasm/reflex.wasm "$out/"
echo "wrote $out/reflex.mjs and reflex.wasm ($commit)"
