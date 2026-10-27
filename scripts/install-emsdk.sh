#!/usr/bin/env bash
# install-emsdk.sh — install the pinned Emscripten SDK (scripts/emsdk-version)
# into <dir> and print the line to source it.
#
#   scripts/install-emsdk.sh <dir>
#   source <dir>/emsdk_env.sh
#
# CI uses it before scripts/build-wasm.sh. The pin is an emsdk release tag;
# change it in scripts/emsdk-version only, and only to a release at least two
# weeks old.
set -euo pipefail

dir=${1:?usage: install-emsdk.sh <dir>}
version=$(tr -d '[:space:]' < "$(dirname "$0")/emsdk-version")
if [ ! -x "$dir/emsdk" ]; then
    git clone --quiet --depth 1 --branch "$version" https://github.com/emscripten-core/emsdk.git "$dir"
fi
"$dir/emsdk" install "$version" >/dev/null
"$dir/emsdk" activate "$version" >/dev/null
echo "source $dir/emsdk_env.sh"
