#!/usr/bin/env bash
set -euox pipefail

version="${1:?usage: update-hugrenv.sh <version>}"
cd "$(dirname "$0")/.."
lockfile="hugrenv.lock"

update_version() {
  local version="$1"
  echo "Replacing version with $version"
  new_json=$(
    jq \
      --arg version "$version" \
      '.version = $version' \
      "$lockfile"
  );
  echo "${new_json}" > "$lockfile"
}

set_hash() {
  local platform="$1"
  local arch="$2"
  local package="$3"
  local hash="$4"
  new_json=$(
    jq \
      --arg platform "$platform" \
      --arg arch "$arch" \
      --arg package "$package" \
      --arg hash "$hash" \
      '.hashes[$platform][$arch][$package] = $hash' \
      "$lockfile"
  );
  echo "${new_json}" > "$lockfile"
}
wipe_hash() {
  echo "Wiping hash for $3 on $1 ($2)"
  set_hash "$1" "$2" "$3" "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
}

extract_hash() {
  sed -n 's/.*got:[[:space:]]*\(sha256-[A-Za-z0-9+\/=]*\).*/\1/p' | tail -n1
}

update_one() {
  local platform="$1"
  local arch="$2"
  local package="$3"
  local found_hash
  local output

  wipe_hash "$platform" "$arch" "$package"

  echo "Finding new hash for $package on $platform ($arch) - this may take some time"

  if output=$(
      nix-build ./hugrenv.nix \
        --argstr platform "$platform" \
        --argstr arch "$arch" \
        --arg packages '["'$package'"]' \
        2>&1
  ); then
    echo "$output"
    echo "Error: expected fixed-output mismatch for ${package} on ${platform} (${arch}), but build succeeded without mismatch"
    exit 1
  fi

  found_hash="$(echo "$output" | extract_hash)"
  [[ -n "$found_hash" ]] || {
      echo "$output";
      echo "failed to extract hash for ${platform} (${arch})";
      exit 1;
  }

  echo "Found hash for $platform ($arch): $found_hash"

  set_hash "$platform" "$arch" "$package" "$found_hash"
}


update_version "$version"

for package in "tket" "llvm"; do
    for platform in "macosx_15_0" "manylinux_2_28"; do
        for arch in "aarch64" "x86_64"; do
            update_one "$platform" "$arch" "$package"
        done
    done
done
