# Changelog

## [0.3.2](https://github.com/Quantinuum/tket2/compare/qis-compiler-v0.3.1...qis-compiler-v0.3.2) (2026-06-19)


### Features

* HUGR extension for global variables ([#1530](https://github.com/Quantinuum/tket2/issues/1530)) ([4209df1](https://github.com/Quantinuum/tket2/commit/4209df1130f092d0936de71154bc98c78745a2ac))

## [0.3.1](https://github.com/Quantinuum/tket2/compare/qis-compiler-v0.3.0...qis-compiler-v0.3.1) (2026-06-15)


### Bug Fixes

* Add missing `tket.measurement` extension to loading registry ([#1692](https://github.com/Quantinuum/tket2/issues/1692)) ([6cdbd54](https://github.com/Quantinuum/tket2/commit/6cdbd54a03fbaf2da370c01647925bf4c5098a58))

## [0.3.0](https://github.com/Quantinuum/tket2/compare/qis-compiler-v0.2.10...qis-compiler-v0.3.0) (2026-06-11)


### ⚠ BREAKING CHANGES

* Updated the HUGR Python dependency to `0.17.1`
* The compiler now targets explicit qsystem platforms. `compile_to_bitcode` and `compile_to_llvm_ir` accept a `platform` argument for `"helios"` or `"sol"`.
* `compile_to_bitcode` and `compile_to_llvm_ir` now require keyword arguments for `opt_level` and `target_triple`.

### Features

* Add `emit_debug` support to `compile_to_bitcode` and `compile_to_llvm_ir` ([#1521](https://github.com/Quantinuum/tket2/issues/1521)) ([db2e530](https://github.com/Quantinuum/tket2/commit/db2e5306deee1b3d8aff7723eb5ae7a91d9d9235))
* Add Helios and Sol qsystem platform support to the compiler ([#1567](https://github.com/Quantinuum/tket2/issues/1567)) ([b60553f](https://github.com/Quantinuum/tket2/commit/b60553fec5e81b698c75916658bae7d1c527907e))
* Add support for HUGRs using the `measurement` extension ([#1558](https://github.com/Quantinuum/tket2/issues/1558)) ([7e35ecf](https://github.com/Quantinuum/tket2/commit/7e35ecf592db05e51e9b4d4b577afc2c93bd291d))
* Use compliant LLVM and TKET artifacts from `hugrverse-env` for wheel builds ([#1471](https://github.com/Quantinuum/tket2/issues/1471)) ([6faaf41](https://github.com/Quantinuum/tket2/commit/6faaf417b76c4aee5d34faa82121832df10a75af))


### Bug Fixes

* Include Helios and Sol qsystem extensions in the compiler registry ([#1646](https://github.com/Quantinuum/tket2/issues/1646)) ([8800257](https://github.com/Quantinuum/tket2/commit/88002572d2f5af63233c2c0179c25da477a5a4e4)), closes [#1645](https://github.com/Quantinuum/tket2/issues/1645)
* Trim trailing NUL bytes from public bitcode ([#1602](https://github.com/Quantinuum/tket2/issues/1602)) ([68dd0c3](https://github.com/Quantinuum/tket2/commit/68dd0c3ad289d3bbe84201dfbbf7ec9a76a5a696))

## [0.2.10](https://github.com/quantinuum/tket2/compare/qis-compiler-v0.2.9...qis-compiler-v0.2.10) (2025-11-10)


### Features

* add GPU lowering to selene-hugr-qis-compiler ([#1169](https://github.com/quantinuum/tket2/issues/1169)) ([bcf1d4c](https://github.com/quantinuum/tket2/commit/bcf1d4c3c7a1dce383fcdd7f4668663b2fbc7c04))

### Bug Fixes

* Fix runtime panic when iterating through arrays of affine/bool types ([hugr#2666](https://github.com/quantinuum/hugr/pull/2666)) ([01b8a8e](https://github.com/quantinuum/tket2/commit/01b8a8e686592459336334849340e70173606550))

## [0.2.9](https://github.com/quantinuum/tket2/compare/qis-compiler-v0.2.8...qis-compiler-v0.2.9) (2025-10-22)


### Bug Fixes

* **qis-compiler:** undo accidental breakage from removing value array ([#1194](https://github.com/quantinuum/tket2/issues/1194)) ([5319856](https://github.com/quantinuum/tket2/commit/53198562dae962d340e53ab07b798510896da01f))

## [0.2.8](https://github.com/quantinuum/tket2/compare/qis-compiler-v0.2.7...qis-compiler-v0.2.8) (2025-10-21)


### Bug Fixes

* clean-up test dependency in qis compiler release workflow ([#1191](https://github.com/quantinuum/tket2/issues/1191)) ([61bc472](https://github.com/quantinuum/tket2/commit/61bc472dfa6a88f5bd7735cd6129aa95e4bd575d))

## [0.2.7](https://github.com/quantinuum/tket2/compare/qis-compiler-v0.2.6...qis-compiler-v0.2.7) (2025-10-20)


### Features

* Port selene-hugr-qis-compiler to tket2 repository ([#1146](https://github.com/quantinuum/tket2/issues/1146)) ([970f3b1](https://github.com/quantinuum/tket2/commit/970f3b1dc8909c7b38071221624564d91b1168cd))
* Switch borrow array lowering from type replacement to llvm ([ab3e020](https://github.com/quantinuum/tket2/commit/ab3e02063ad8794a93c255f7c9198a16e73c572b))

## [0.2.6](https://github.com/quantinuum/selene/compare/selene-hugr-qis-compiler-v0.2.5...selene-hugr-qis-compiler-v0.2.6) (2025-09-22)


### Bug Fixes

* **compiler:** error when entrypoint has arguments ([#84](https://github.com/quantinuum/selene/issues/84)) ([604b131](https://github.com/quantinuum/selene/commit/604b1311b96593609e699a6bb8251ad3c952ebdb))

## [0.2.5](https://github.com/quantinuum/selene/compare/selene-hugr-qis-compiler-v0.2.4...selene-hugr-qis-compiler-v0.2.5) (2025-09-19)


### Features

* **compiler:** Bump tket version; add wasm + gpu to the hugr-qis registry ([c69155d](https://github.com/quantinuum/selene/commit/c69155d9717e942c6c67065dbf47cdb156542689))

## [0.2.4](https://github.com/quantinuum/selene/compare/selene-hugr-qis-compiler-v0.2.3...selene-hugr-qis-compiler-v0.2.4) (2025-08-28)


### Bug Fixes

* **compiler:** update tket-qystem to fix CZ bug ([#78](https://github.com/quantinuum/selene/issues/78)) ([3991f11](https://github.com/quantinuum/selene/commit/3991f11a73d8ceebf0346a8c43248fde73e1b549))

## [0.2.3](https://github.com/quantinuum/selene/compare/selene-hugr-qis-compiler-v0.2.2...selene-hugr-qis-compiler-v0.2.3) (2025-08-28)


### Features

* Emit a nicer error when trying to emulate unsupported pytket ops ([#72](https://github.com/quantinuum/selene/issues/72)) ([d88a28a](https://github.com/quantinuum/selene/commit/d88a28a827d15fb2fcbc036964452fdcfd7b1cd8))

## [0.2.2](https://github.com/quantinuum/selene/compare/selene-hugr-qis-compiler-v0.2.1...selene-hugr-qis-compiler-v0.2.2) (2025-08-21)


### Features

* update to tket-qsystem 0.20 ([#66](https://github.com/quantinuum/selene/issues/66)) ([7191b07](https://github.com/quantinuum/selene/commit/7191b07c00571c0298b3cfc334058d3e649fe377))
