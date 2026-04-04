{
  pkgs ? import <nixpkgs> {},
  platform ? if pkgs.stdenv.isDarwin then "macosx_15_0" else "manylinux_2_28",
  arch ? if pkgs.stdenv.isAarch64 then "aarch64" else "x86_64",
}:
let
  sources = builtins.fromJSON (builtins.readFile ./hugrenv.lock);
  version = sources.version;
  hashes = sources.hashes;
  platform_arch = "${platform}_${arch}";

  url = "https://github.com/Quantinuum/hugrverse-env/releases/download/v${version}/hugrenv-tket-${platform_arch}.tar.gz";
  sha256 = hashes.${platform}.${arch};
in {
  version = version;
  url = url;
  path = pkgs.fetchzip { inherit url sha256; };
}
