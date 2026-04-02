{ pkgs, lib, inputs, ... }:
let
  llvmVersion = "21";
  llvmPackages = pkgs."llvmPackages_${llvmVersion}";
  versionInfo = builtins.splitVersion llvmPackages.release_version;
  llvmVersionMajor = builtins.elemAt versionInfo 0;
  llvmVersionMinor = builtins.elemAt versionInfo 1;
  hugrenv = pkgs.callPackage ./hugrenv.nix {};
in {
  # https://devenv.sh/packages/
  # on macos frameworks have to be explicitly specified
  # otherwise a linker error occurs on rust packages
  packages = [
    pkgs.just
    pkgs.cargo-insta
    pkgs.cargo-nextest

    # These are required to be able to link to llvm.
    pkgs.libffi
    pkgs.libxml2
    pkgs.zlib
    pkgs.ncurses
    pkgs.stdenv.cc.cc.lib
    pkgs.conan
    # cmake is needed for conan to build packages from source when
    # prebuilt binaries aren't available for Nix's clang version.
    pkgs.cmake

  ] ++ lib.optionals pkgs.stdenv.isDarwin [
    pkgs.xz
  ];

  enterShell = ''
    cargo --version
    python --version
    uv --version
    export LD_LIBRARY_PATH="${hugrenv}/lib:${hugrenv}/lib64:$LD_LIBRARY_PATH"
  '';

  env = {
    "LLVM_SYS_${llvmVersionMajor}${llvmVersionMinor}_PREFIX" = "${llvmPackages.libllvm.dev}";
    "TKET_C_API_PATH" = "${hugrenv}";
    "LIBCLANG_PATH" = "${pkgs.libclang.lib}/lib";
  };

  # https://devenv.sh/languages/

  languages.rust = {
    enable = true;
    channel = "stable";
    components = [ "rustc" "cargo" "clippy" "rustfmt" "rust-analyzer" ];
  };

  languages.python = {
    enable = true;
    uv = {
      enable = true;
      sync.enable = true;
    };
    venv.enable = true;
  };


}
