{ pkgs, lib, inputs, config, ... }:
let
  hugrenv = config.hugrenv.finalPackage;
  # Let just evaluate its toolchain pin rather than parsing the justfile.
  nightlyToolchain = builtins.readFile (pkgs.runCommand "tket-nightly-toolchain" {
    nativeBuildInputs = [ pkgs.just ];
  } ''
    just --justfile ${./justfile} --evaluate nightly_toolchain > "$out"
  '');
in {

  options.hugrenv = {
    # Disable to use Nix-provided LLVM 21 and libclang (for example on NixOS).
    llvm.enable = lib.mkEnableOption "LLVM from hugrenv" // { default = true; };
    package = lib.mkOption {
      type = lib.types.package;
      default = pkgs.callPackage ./hugrenv.nix {
        packages = [ "tket" ];
      };
    };
    finalPackage = lib.mkOption {
      internal = true;
      type = lib.types.package;
    };
  };

  config = {
    hugrenv.finalPackage = let
      pkg = config.hugrenv.package;
    in if config.hugrenv.llvm.enable
      then pkg.override (old: { packages = old.packages ++ [ "llvm" ]; })
      else pkg;
    # https://devenv.sh/packages/
    # on macos frameworks have to be explicitly specified
    # otherwise a linker error occurs on rust packages
    packages = [
      hugrenv
      pkgs.just
      pkgs.cargo-insta
      pkgs.cargo-nextest

      # These are required to be able to link to llvm.
      pkgs.libffi
      # used to override jemalloc-sys to use nixpkgs' jemalloc
      # instead of building with cmake (and requiring reduced hardening)
      pkgs.jemalloc
    ] ++ lib.optionals pkgs.stdenv.isDarwin [
      pkgs.xz
    ] ++ lib.optionals (!config.hugrenv.llvm.enable) [
      pkgs.zlib
      pkgs.libxml2
    ];

    enterShell = ''
      cargo --version
      python --version
      uv --version
    '';

    env = {
      "LLVM_SYS_211_PREFIX" = if config.hugrenv.llvm.enable
        then "${hugrenv}"
        else "${pkgs.llvmPackages_21.llvm.dev}";
      "JEMALLOC_OVERRIDE" =
        if pkgs.stdenv.isDarwin
        then "${pkgs.jemalloc}/lib/libjemalloc.dylib"
        else "${pkgs.jemalloc}/lib/libjemalloc.so";
      "TKET_C_API_PATH" = "${hugrenv}";
      "LIBCLANG_PATH" = if config.hugrenv.llvm.enable
        then "${hugrenv}/lib"
        else "${pkgs.llvmPackages_21.libclang.lib}/lib";
    };

    # https://devenv.sh/languages/

    languages.rust = {
      enable = true;
      channel = "stable";
      components = [ "rustc" "cargo" "clippy" "rustfmt" "rust-analyzer" ];
    };

    # Nightly toolchain required for pg-libs' `unstable_simd` feature
    profiles.nightly.module = {
      languages.rust = {
        channel = "nightly";
        version = lib.removePrefix "nightly-" nightlyToolchain;
        components = [ "rustc" "cargo" "clippy" "rustfmt" "rust-analyzer" ];
      };
    };

    languages.python = {
      enable = true;
      uv = {
        enable = true;
        sync.enable = true;
      };
      venv.enable = true;
    };

  };

}
