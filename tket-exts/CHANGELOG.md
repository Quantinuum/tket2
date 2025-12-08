# Changelog

## [0.12.0](https://github.com/quantinuum/tket2/compare/tket-exts-v0.11.0...tket-exts-v0.12.0) (2025-10-20)


### ⚠ BREAKING CHANGES

* Require hugr>=0.14 to support borrow arrays


### Features

* Definition of extension ops for modifiers and global phases ([#1137](https://github.com/quantinuum/tket2/issues/1137)) ([ffc507b](https://github.com/quantinuum/tket2/commit/ffc507b22c462aab3d5ddf837b86e5ac3325e731))
* Add `random_advance` to random extensions ([#1170](https://github.com/quantinuum/tket2/issues/1170)) ([eed16e8](https://github.com/quantinuum/tket2/commit/eed16e8c57541ea23af6241290aa47d37583cfc4))

## [0.11.0](https://github.com/quantinuum/tket2/compare/tket-exts-v0.10.1...tket-exts-v0.11.0) (2025-09-12)


### ⚠ BREAKING CHANGES

* Move `add_const_module` from `WasmOpBuilder` into new trait, `ConstWasmBuilder`. Update descriptions of wasm ops json.
* This is a breaking change to the WASM extension and its serialised representation

### Features

* Add gpu module ([#1090](https://github.com/quantinuum/tket2/issues/1090)) ([6f035f6](https://github.com/quantinuum/tket2/commit/6f035f6ce1e281310fd6903bffb3a98099ee4ae0))
* Explicit exports for tket_exts ops and types ([#1046](https://github.com/quantinuum/tket2/issues/1046)) ([a32873e](https://github.com/quantinuum/tket2/commit/a32873e3543b7d77f3bed08016485ab292f5204a))
* Update WASM extension ([#1047](https://github.com/quantinuum/tket2/issues/1047)) ([5f9560d](https://github.com/quantinuum/tket2/commit/5f9560de30b797f718b2071f0615f3f3d55205e1))


### Code Refactoring

* Factor out wasm extension code into compute module ([#1089](https://github.com/quantinuum/tket2/issues/1089)) ([9ea1b1e](https://github.com/quantinuum/tket2/commit/9ea1b1ecab84a72fbf545c4f8871f5b70f0e9776))

## [0.10.1](https://github.com/quantinuum/tket2/compare/tket-exts-v0.10.0...tket-exts-v0.10.1) (2025-08-19)


### Documentation

* Update README badges ([#1004](https://github.com/quantinuum/tket2/issues/1004)) ([d609bf5](https://github.com/quantinuum/tket2/commit/d609bf5f65af3cfe3ac44a16dfd4ef1bcacd5643))

## [0.10.0](https://github.com/quantinuum/tket2/compare/tket-exts-v0.9.2...tket-exts-v0.10.0) (2025-07-29)


### ⚠ BREAKING CHANGES

* Renamed the `tket2.*` HUGR extensions to `tket.*`
* Libraries renamed from `tket2*` to `tket*`

### Features

* **py:** update hugr-py dependency to 0.13 ([#996](https://github.com/quantinuum/tket2/issues/996)) ([1bf4c70](https://github.com/quantinuum/tket2/commit/1bf4c70788693d357b3cb2dcdbe2c721951da2a5))
* Rename `tket2.*` HUGR extensions to `tket.*` ([#988](https://github.com/quantinuum/tket2/issues/988)) ([c5279c5](https://github.com/quantinuum/tket2/commit/c5279c55c1287980ff18c0bfdf360f69be5f345f))
* Rename tket2* libs to tket* ([#987](https://github.com/quantinuum/tket2/issues/987)) ([450f06a](https://github.com/quantinuum/tket2/commit/450f06ae6b2d7472ad33418299479709e307919c))

## [0.9.2](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.9.1...tket2-exts-v0.9.2) (2025-07-08)


### Features

* add qsystem op for measure leaked ([#924](https://github.com/quantinuum/tket2/issues/924)) ([38d1c6f](https://github.com/quantinuum/tket2/commit/38d1c6f51131b414e1000e5f63a66aae32a36f28))

## [0.9.1](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.8.0...tket2-exts-v0.9.1) (2025-06-19)


### Features

* Add V and Vdg to quantum extension. ([#889](https://github.com/quantinuum/tket2/issues/889)) ([a8f1569](https://github.com/quantinuum/tket2/commit/a8f156930ff3afc35af15a2afdd24fc65c8409b5))


### Bug Fixes

* Bump version of quantum extension. ([#894](https://github.com/quantinuum/tket2/issues/894)) ([beddb99](https://github.com/quantinuum/tket2/commit/beddb99763e444c0c72853fc6111d4805e4625ea))

## [0.8.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.7.0...tket2-exts-v0.8.0) (2025-05-22)


### ⚠ BREAKING CHANGES

* (`tket2.bool` extension) `BoolOp::bool_to_sum` / `BoolOp::sum_to_bool` renamed to `BoolOp::read` / `BoolOp::make_opaque` `Tk2Op::MeasureFree` now returns a `tket2.bool` (`tket2-hseries.qsystem` extension) `QSystemOp:Measure` and `QSystemOp:MeasureReset` now return `tket2.bool`s
* **tket2-hseries:** `QSystemOpBuilder` gained supertrait `ArrayOpBuilder`

### Features

* Add `ReplaceBoolPass` ([#854](https://github.com/quantinuum/tket2/issues/854)) ([5ae0ab9](https://github.com/quantinuum/tket2/commit/5ae0ab9d7046a73019bf8a7bc436a576bece1fa0))
* **tket2-hseries:** insert RuntimeBarrier across qubits in a Barrier ([#866](https://github.com/quantinuum/tket2/issues/866)) ([6bcc9d6](https://github.com/quantinuum/tket2/commit/6bcc9d62d30accca91edc6255d42ec300763c263))

## [0.7.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.6.0...tket2-exts-v0.7.0) (2025-05-16)


### ⚠ BREAKING CHANGES

* bump to hugr 0.20
* **hseries:** ZZMax removed from Qsystem extension. Use ZZPhase(pi/2).

### Features

* Add `tket2.bool` extension ([#823](https://github.com/quantinuum/tket2/issues/823)) ([8818d2f](https://github.com/quantinuum/tket2/commit/8818d2f122ec3911366d02c18e347752a427fad9))
* Add debug extension with state result op ([#843](https://github.com/quantinuum/tket2/issues/843)) ([64bbe88](https://github.com/quantinuum/tket2/commit/64bbe88ddd0a214044d2636f3d4bd8262d6b05f5)), closes [#832](https://github.com/quantinuum/tket2/issues/832)
* **hseries:** remove ZZMax operation from Qsystem extension ([#852](https://github.com/quantinuum/tket2/issues/852)) ([b488125](https://github.com/quantinuum/tket2/commit/b4881256b2d6a5c21c1d7a69d91384c5d2cc9905))


### Miscellaneous Chores

* Bump to hugr-0.20 ([#862](https://github.com/quantinuum/tket2/issues/862)) ([652a7d0](https://github.com/quantinuum/tket2/commit/652a7d0b039bca62407f16f7548204e97f92ef5a))

## [0.6.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.5.1...tket2-exts-v0.6.0) (2025-03-18)


### ⚠ BREAKING CHANGES

* Removed `OrderInZones` operation from `tket2.qsystem.utils`.


### Removed

* Removed `OrderInZones` operation from `tket2.qsystem.utils`. ([#797](https://github.com/quantinuum/tket2/issues/797)) ([8c3ee89](https://github.com/quantinuum/tket2/commit/8c3ee8971b9f095ccdb0522cf0751a2ff20b89a5))


## [0.5.1](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.5.0...tket2-exts-v0.5.1) (2025-03-06)


### Features

* Add order_in_zones extension op ([3ec7f5d](https://github.com/quantinuum/tket2/commit/3ec7f5d5e0a7d07254e1b09976cddea98cd83702))

## [0.5.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.4.0...tket2-exts-v0.5.0) (2025-03-04)


### ⚠ BREAKING CHANGES

* To be compatible with Guppy's convention of implicitly returning `self` as the second value of the tuple, the following signatures are updated: ```diff
    -    /// `fn random_int(RNGContext) -> (RNGContext, u32)`
    +   /// `fn random_int(RNGContext) -> (u32, RNGContext)`

### Bug Fixes

* remove type argument from `RNGContext` type, swap returns ([#786](https://github.com/quantinuum/tket2/issues/786)) ([633ebd7](https://github.com/quantinuum/tket2/commit/633ebd74d71ba81f5b71d6db757b08ea3c959a5d))

## [0.4.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.3.0...tket2-exts-v0.4.0) (2025-02-20)


### ⚠ BREAKING CHANGES

* **tket2-hseries:** The signature of `QSystemOp::LazyMeasure` is changed to consume its qubit.

### Features

* add "tket2.qsystem.random" extension ([#779](https://github.com/quantinuum/tket2/issues/779)) ([f4901ee](https://github.com/quantinuum/tket2/commit/f4901eed22f8e66ca5ea3ccb8d72ead134ff8001))
* add a `tket2.qsystem.utils` extension with `GetCurrentShot` ([#772](https://github.com/quantinuum/tket2/issues/772)) ([175a02d](https://github.com/quantinuum/tket2/commit/175a02da2ce8a0065c265cdae7518c1b5284cff3)), closes [#767](https://github.com/quantinuum/tket2/issues/767)
* **tket2-hseries:** Add `tket2.wasm` extension ([#737](https://github.com/quantinuum/tket2/issues/737)) ([34bdc21](https://github.com/quantinuum/tket2/commit/34bdc218b5e9bf334830873e847935dea0053242))
* **tket2-hseries:** Redefine `QSystemOp::LazyMeasure` and introduce `QSystemOp::LazyMeasureReset` ([#741](https://github.com/quantinuum/tket2/issues/741)) ([1f126c0](https://github.com/quantinuum/tket2/commit/1f126c0a4f7686fa6941a05aa28228786baac6d1))
* update measurement and alloc operations ([#702](https://github.com/quantinuum/tket2/issues/702)) ([a7a0201](https://github.com/quantinuum/tket2/commit/a7a020116f42bfeb89c356d08816a2f3ce1b5226))


### Bug Fixes

* remove unicode pi symbols in rotation extension ([#743](https://github.com/quantinuum/tket2/issues/743)) ([b3ed351](https://github.com/quantinuum/tket2/commit/b3ed35108d5fe93c3aa8101084b695470c488a30))


### Documentation

* docstring capitalisation ([#686](https://github.com/quantinuum/tket2/issues/686)) ([e18f921](https://github.com/quantinuum/tket2/commit/e18f921903953dc6a033ef697092f80a99a142b0))


## [0.3.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.2.0...tket2-exts-v0.3.0) (2024-12-16)


### ⚠ BREAKING CHANGES

* Updated `hugr` rust dependency to `0.14.0`. Extension are now defined in `hugr 0.14` style.

### Miscellaneous Chores

* Update to next version of hugr ([#720](https://github.com/quantinuum/tket2/issues/720)) ([4a3a5a5](https://github.com/quantinuum/tket2/commit/4a3a5a5e38252d4ee709e7e97bb5a1e90bd9fff4))

## [0.2.0](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.1.1...tket2-exts-v0.2.0) (2024-12-02)


### ⚠ BREAKING CHANGES

* all references to "hseries" including extension, modules and structs renamed to "qsystem"
* hseries qalloc op replaced with fallible TryQalloc

### Features

* update measurement and alloc operations ([#702](https://github.com/quantinuum/tket2/issues/702)) ([a7a0201](https://github.com/quantinuum/tket2/commit/a7a020116f42bfeb89c356d08816a2f3ce1b5226))


### Code Refactoring

* rename hseries to qsystem ([#703](https://github.com/quantinuum/tket2/issues/703)) ([1e90173](https://github.com/quantinuum/tket2/commit/1e90173872e73c44a6321fe400ae6f2e4e115220))

## [0.1.1](https://github.com/quantinuum/tket2/compare/tket2-exts-v0.1.0...tket2-exts-v0.1.1) (2024-11-05)


### Documentation

* docstring capitalisation ([#686](https://github.com/quantinuum/tket2/issues/686)) ([e18f921](https://github.com/quantinuum/tket2/commit/e18f921903953dc6a033ef697092f80a99a142b0))

## 0.1.0 (2024-11-06)


### Features

* Add tket2 extension definitions.
