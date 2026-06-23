{ pkgs, lib, inputs, config, ... }:
let hugrenv = config.hugrenv.package;
in {

  options.hugrenv.package = lib.mkOption {
    type = lib.types.package;
    default =  pkgs.callPackage ./hugrenv.nix {
        packages = ["tket" "llvm"];
    };
  };

  config = {
  # https://devenv.sh/packages/
  # on macos frameworks have to be explicitly specified
  # otherwise a linker error occurs on rust packages
  packages = [
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
  ];

  enterShell = ''
    cargo --version
    python --version
    uv --version
    # append hugrenv to bin and lib paths
    export PATH="${hugrenv}/bin:$PATH"
    # if macos use DYLD_LIBRARY_PATH instead of LD_LIBRARY_PATH
    if [ "$(uname)" = "Darwin" ]; then
      export DYLD_LIBRARY_PATH="${hugrenv}/lib:${hugrenv}/lib64:${pkgs.stdenv.cc.cc.lib}/lib:$DYLD_LIBRARY_PATH"
    else
      export LD_LIBRARY_PATH="${hugrenv}/lib:${hugrenv}/lib64:${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"
    fi
  '';

  env = {
    "LLVM_SYS_211_PREFIX" = "${hugrenv}";
    "TKET_C_API_PATH" = "${hugrenv}";
    "LIBCLANG_PATH" = "${hugrenv}/lib";
    "JEMALLOC_OVERRIDE" =
      if pkgs.stdenv.isDarwin
      then "${pkgs.jemalloc}/lib/libjemalloc.dylib"
      else "${pkgs.jemalloc}/lib/libjemalloc.so";
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

  };

}
