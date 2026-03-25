{ pkgs }:
let
  hugrenvVersion = "0.3.0";
  currentPlatform = if pkgs.stdenv.isDarwin then "macosx_15_0" else "manylinux_2_28";
  currentArch = if pkgs.stdenv.isAarch64 then "aarch64" else "x86_64";
  currentPlatformArch = "${currentPlatform}_${currentArch}";

  hugrenvUrl = "https://github.com/Quantinuum/hugrverse-env/releases/download/v${hugrenvVersion}/hugrenv-tket-${currentPlatformArch}.tar.gz";
  hugrenvHashes = {
    "macosx_15_0_aarch64"    = "sha256-IrapjnU2H3Eb06sxXr9M8KpL0qjBwqot9QUudBBJYvA=";
    "macosx_15_0_x86_64"     = "sha256-9Tyrsrxz8/LMR/q2jwmdTO77tgk1NCUFrc5/ilCjgtM=";
    "manylinux_2_28_aarch64" = "sha256-OdHJj3pnxdet5dpEV4aVvom9pb6lZ1koyyJX0vHVns8=";
    "manylinux_2_28_x86_64"  = "sha256-YAZ1kQK4MU3rrgXiyopTeSrx4cBhVEAxJyxoB5DIDqM=";
  };
in pkgs.fetchzip {
  url = hugrenvUrl;
  sha256 = hugrenvHashes."${currentPlatformArch}";
}
