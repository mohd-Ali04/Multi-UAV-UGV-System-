#!/usr/bin/env bash
# Build the versioned, runnable export:  scripts/make_release.sh 1.0.0
# Produces release/tins8-course-<version>.tar.gz, its .sha256, and MANIFEST.sha256 inside the archive.
# Content = every file tracked by git (source, interfaces, world, models, meshes, configs, launch,
# scripts, GUI, RViz config, dependency manifests, guides).  build/install/log are never included.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ver="${1:?usage: make_release.sh <version>}"
name="tins8-course-$ver"
out="release"; stage="$out/$name"
rm -rf "$stage"; mkdir -p "$stage"
git ls-files -z | xargs -0 -I{} cp --parents {} "$stage/"
(cd "$stage" && find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | xargs -0 sha256sum > MANIFEST.sha256)
tar -C "$out" -czf "$out/$name.tar.gz" "$name"
(cd "$out" && sha256sum "$name.tar.gz" > "$name.tar.gz.sha256")
rm -rf "$stage"
echo "$(cat "$out/$name.tar.gz.sha256")"
echo "files: $(tar -tzf "$out/$name.tar.gz" | grep -vc '/$')"
