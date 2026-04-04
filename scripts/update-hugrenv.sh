#!/usr/bin/env bash
set -euo pipefail

version="${1:?usage: update-hugrenv.sh <version>}"
cd "$(dirname "$0")"
cd ..
lockfile="hugrenv.lock"

update_version() {
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
  local hash="$3"
  new_json=$(
    jq \
      --arg platform "$platform" \
      --arg arch "$arch" \
      --arg hash "$hash" \
      '.hashes[$platform][$arch] = $hash' \
      "$lockfile"
  );
  echo "${new_json}" > "$lockfile"
}
wipe_hash() {
  echo "Wiping hash for $1 ($2)"
  set_hash "$1" "$2" "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="
}

extract_hash() {
  sed -n 's/.*got:[[:space:]]*\(sha256-[A-Za-z0-9+\/=]*\).*/\1/p' | tail -n1
}

update_one() {
  local platform="$1"
  local arch="$2"
  local found_hash
  local output

  wipe_hash "$platform" "$arch"

  echo "Finding new hash for $platform ($arch) - this may take some time"

  if output=$(nix-build ./hugrenv.nix --argstr platform "$platform" --argstr arch "$arch" 2>&1); then
    echo "$output"
    echo "Error: expected fixed-output mismatch for $key"
    exit 1
  fi

  found_hash="$(echo "$output" | extract_hash)"
  [[ -n "$found_hash" ]] || {
      echo "$output";
      echo "failed to extract hash for ${platform} (${arch})";
      exit 1;
  }

  echo "Found hash for $platform ($arch): $found_hash"

  set_hash "$platform" "$arch" "$found_hash"
}

update_version
update_one macosx_15_0 aarch64
update_one macosx_15_0 x86_64
update_one manylinux_2_28 aarch64
update_one manylinux_2_28 x86_64
