{ pkgs }:
let
  hugrenvVersion = "0.3.1";
  currentPlatform = if pkgs.stdenv.isDarwin then "macosx_15_0" else "manylinux_2_28";
  currentArch = if pkgs.stdenv.isAarch64 then "aarch64" else "x86_64";
  currentPlatformArch = "${currentPlatform}_${currentArch}";

  hugrenvUrl = "https://github.com/Quantinuum/hugrverse-env/releases/download/v${hugrenvVersion}/hugrenv-tket-${currentPlatformArch}.tar.gz";
  hugrenvHashes = {
    "macosx_15_0_aarch64"    = "sha256-4CZUc9GWnhAej8lMgCFMNiUb98EUzQOklWGQ30ENQ4k=";
    "macosx_15_0_x86_64"     = "sha256-tYEjqF0s/30G8WZTJ0UPYpcnFM6pQOHFH7giUHScRaw=";
    "manylinux_2_28_aarch64" = "sha256-NFcgtBxx4CURQYQjrR4VMn6LCLkT99OJqnRP1ilil0k=";
    "manylinux_2_28_x86_64"  = "sha256-cj2kx3mCgXe5km9n6MI56S8IySQMX9ZcMsZ3a8T0kh0=";
  };
in pkgs.fetchzip {
  url = hugrenvUrl;
  sha256 = hugrenvHashes."${currentPlatformArch}";
}
