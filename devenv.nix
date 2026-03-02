{ pkgs, lib, inputs, ... }:
let
  llvmVersion = "21";
  llvmPackages = pkgs."llvmPackages_${llvmVersion}";
  versionInfo = builtins.splitVersion llvmPackages.release_version;
  llvmVersionMajor = builtins.elemAt versionInfo 0;
  llvmVersionMinor = builtins.elemAt versionInfo 1;
in
{
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

  ] ++ lib.optionals pkgs.stdenv.isDarwin [
    pkgs.xz
  ];

  # Required for uv sync to work
  tasks."tket2:conan_profile_detect" = {
    exec = ''
      conan profile detect --exist-ok
    '';
    before = [ "devenv:python:uv" ];
  };

  enterShell = ''
    cargo --version
    python --version
    uv --version
  '';

  env = {
    "LLVM_SYS_${llvmVersionMajor}${llvmVersionMinor}_PREFIX" = "${llvmPackages.libllvm.dev}";
    "LIBCLANG_PATH" = "${pkgs.libclang.lib}/lib";
    # hardening removed due its impact on tikv-jemalloc-sys build,
    # as depended upon by tikv-jemalloc-sys
    # See https://github.com/tikv/jemallocator/issues/108
    "NIX_HARDENING_ENABLE" = "";
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
