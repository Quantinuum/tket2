# Changelog

## [0.12.13](https://github.com/Quantinuum/tket2/compare/tket-py-v0.12.12...tket-py-v0.12.13) (2025-12-10)


### Features

* Allow running arbitrary serializable pytket passes on hugrs ([#1266](https://github.com/Quantinuum/tket2/issues/1266)) ([24875e0](https://github.com/Quantinuum/tket2/commit/24875e06121a6929e842cc3f5c78126fc8bffc6f))
* do constant folding by default in NormalizeGuppy ([#1309](https://github.com/Quantinuum/tket2/issues/1309)) ([3838c49](https://github.com/Quantinuum/tket2/commit/3838c4903077ca926a81141e10b18a984c4fbcf7))
* implement `ComposablePass` for normalize Guppy pass ([#1286](https://github.com/Quantinuum/tket2/issues/1286)) ([d72d84d](https://github.com/Quantinuum/tket2/commit/d72d84d55b22ed574fe15f322304d54c5d445d1a))
* implement `ComposeablePass` interface for any serializable pytket pass. ([#1269](https://github.com/Quantinuum/tket2/issues/1269)) ([71cb1f2](https://github.com/Quantinuum/tket2/commit/71cb1f2354b9e5058ff3c4ebe3e932e219105cbc))


### Bug Fixes

* update incomplete API docs for passes module ([#1308](https://github.com/Quantinuum/tket2/issues/1308)) ([0276ba2](https://github.com/Quantinuum/tket2/commit/0276ba201c38dbbccc92c237d5947b3f429a2a1a))
* update incorrect version info in notebook ([#1312](https://github.com/Quantinuum/tket2/issues/1312)) ([af160ed](https://github.com/Quantinuum/tket2/commit/af160ed403edd836161ef46477423a31c15af4de))
* Use qsystem extensions on Tk2Circuit.from_bytes/str ([#1296](https://github.com/Quantinuum/tket2/issues/1296)) ([df9f52a](https://github.com/Quantinuum/tket2/commit/df9f52a1d8cb0047fc4732beddfaeb2aa9b43d8b))


### Documentation

* add notebook on optimization of Guppy programs ([#1287](https://github.com/Quantinuum/tket2/issues/1287)) ([e82f959](https://github.com/Quantinuum/tket2/commit/e82f959c5b6a32ab4dcd0c60fc685e89e941d7ae))

## [0.12.12](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.11...tket-py-v0.12.12) (2025-11-18)


### Bug Fixes

* Support MacOs ≥13.0 on Apple-Silicon (https://github.com/quantinuum/tket2/pull/1264) ([07667a7](https://github.com/quantinuum/tket2/commit/07667a765d21f488f7e40f8132f0fdc26c6cba80))


### Documentation

* Document missing passes and types modules with sphinx (https://github.com/quantinuum/tket2/pull/1259) ([07667a7](https://github.com/quantinuum/tket2/commit/07667a765d21f488f7e40f8132f0fdc26c6cba80))

## [0.12.11](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.10...tket-py-v0.12.11) (2025-11-13)


### Bug Fixes

* **ci:** Make wheels compatible with MacOS 15.0 (https://github.com/quantinuum/tket2/issues/1248) ([49c5996](https://github.com/quantinuum/tket2/commit/49c599630a8e1a45f5a1e134a6783f787a5dbed6))
* Don't use opgroup in pytket barrier encoding (https://github.com/quantinuum/tket2/issues/1251) ([49c5996](https://github.com/quantinuum/tket2/commit/49c599630a8e1a45f5a1e134a6783f787a5dbed6))
* guppy_to_circuit always returns num_operations = 0 (https://github.com/quantinuum/tket2/issues/1200) ([49c5996](https://github.com/quantinuum/tket2/commit/49c599630a8e1a45f5a1e134a6783f787a5dbed6))
* **pytket-decoder:** Avoid QAllocating and immediately freeing qubits ([#1256](https://github.com/quantinuum/tket2/issues/1256)) ([228ff52](https://github.com/quantinuum/tket2/commit/228ff52c890fd3ad0a461d06abe88c64205c179f))

## [0.12.10](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.9...tket-py-v0.12.10) (2025-11-11)


### Bug Fixes

* Fix wheel builds for macos ([46619f7](https://github.com/quantinuum/tket2/commit/46619f781fe4e75d0eb7799221e547519f78f631))

## [0.12.9](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.8...tket-py-v0.12.9) (2025-11-11)


### Features

* Add bindings for tket1-passes from python ([[#1225](https://github.com/quantinuum/tket2/issues/1225)](https://github.com/quantinuum/tket2/pull/1225)) ([fce8084](https://github.com/quantinuum/tket2/commit/fce8084d84ad1b8c31fc460f710093a6c54710ed))

## [0.12.8](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.7...tket-py-v0.12.8) (2025-10-20)

### Documentation

* Tket-py notebook cleanup ([9d6f241](https://github.com/quantinuum/tket2/commit/9d6f241382f40643150836a2dcc9bdc7cd9eeb96))

## [0.12.7](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.6...tket-py-v0.12.7) (2025-09-12)


### Bug Fixes

* Add gpu extension to extensions reexposed by tket-py ([#1102](https://github.com/quantinuum/tket2/issues/1102)) ([f826fc5](https://github.com/quantinuum/tket2/commit/f826fc5ab81927d85aaea42a7c4e8fc9e88e55d8))
* Add mypy ignore annotation ([#1103](https://github.com/quantinuum/tket2/issues/1103)) ([9f16212](https://github.com/quantinuum/tket2/commit/9f162127996856506db3ea20f7ff9a1c038fe0f2))


### Documentation

* add Python API docs ([#1087](https://github.com/quantinuum/tket2/issues/1087)) ([e94b168](https://github.com/quantinuum/tket2/commit/e94b16878328c6a63c276cd52f36e2c565fe0417))

## [0.12.6](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.5...tket-py-v0.12.6) (2025-08-29)


### Bug Fixes

* Incorrect ZZMax decoding  from pytket ([#1083](https://github.com/quantinuum/tket2/issues/1083)) ([4752663](https://github.com/quantinuum/tket2/commit/47526636f3d09aa372bce8768bbd802cc51bbe6f))

## [0.12.5](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.4...tket-py-v0.12.5) (2025-08-26)


### Features

* Capture pytket's output permutation explicitly in the hugr connectivity ([#1075](https://github.com/quantinuum/tket2/issues/1075)) ([5fc9614](https://github.com/quantinuum/tket2/commit/5fc96147b4aa830ca347834c7b3cf4a35ce91764))

## [0.12.4](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.3...tket-py-v0.12.4) (2025-08-25)


### Bug Fixes

* Pytket barrier operations not being decoded ([#1069](https://github.com/quantinuum/tket2/issues/1069)) ([4b90ffd](https://github.com/quantinuum/tket2/commit/4b90ffdc7de6ec696d5db5e946b417bf8ff71878))

## [0.12.3](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.2...tket-py-v0.12.3) (2025-08-22)


### Features

* Explicit exports for tket_exts ops and types ([#1046](https://github.com/quantinuum/tket2/issues/1046)) ([a32873e](https://github.com/quantinuum/tket2/commit/a32873e3543b7d77f3bed08016485ab292f5204a))


### Bug Fixes

* Fix erroneous parameters being decoded from pytket for qsystem gates ([#1061](https://github.com/quantinuum/tket2/issues/1061)) ([cd42644](https://github.com/quantinuum/tket2/commit/cd42644ccd50a60795e527efdfbe0727344db373))

## [0.12.2](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.1...tket-py-v0.12.2) (2025-08-19)


### Features

* Define a wire tracker for the new pytket decoder ([#1036](https://github.com/quantinuum/tket2/issues/1036)) ([2466ee2](https://github.com/quantinuum/tket2/commit/2466ee26ab75b4e62136bd151d55f25ce8d1adbd))
* Support qsystem native operations when loading pytket circuits ([[#1041](https://github.com/quantinuum/tket2/issues/1041)](https://github.com/quantinuum/tket2/issues/1041)) ([88c5c79](https://github.com/quantinuum/tket2/commit/88c5c7920fe954d59d8dc8460939ba0f29f306d4))
* **tket-py:** Create BadgerOptimiser from arbitrary Rewriters ([#1022](https://github.com/quantinuum/tket2/issues/1022)) ([a975c1d](https://github.com/quantinuum/tket2/commit/a975c1db0ca1f586cd0e64bbd6054f8aa6ed62b9)), closes [#1021](https://github.com/quantinuum/tket2/issues/1021)


### Documentation

* Update README badges ([#1004](https://github.com/quantinuum/tket2/issues/1004)) ([d609bf5](https://github.com/quantinuum/tket2/commit/d609bf5f65af3cfe3ac44a16dfd4ef1bcacd5643))

## [0.12.1](https://github.com/quantinuum/tket2/compare/tket-py-v0.12.0...tket-py-v0.12.1) (2025-07-29)


### Bug Fixes

* **py:** update tket-py dependencies on other workspace packages ([#1000](https://github.com/quantinuum/tket2/issues/1000)) ([4fab27b](https://github.com/quantinuum/tket2/commit/4fab27bdf2eed8a81a25de5146bffe50337f3259))

## [0.12.0](https://github.com/quantinuum/tket2/compare/tket-py-v0.11.1...tket-py-v0.12.0) (2025-07-29)


### ⚠ BREAKING CHANGES

* Renamed the `tket2.*` HUGR extensions to `tket.*`
* Libraries renamed from `tket2*` to `tket*`

### Features

* **py:** update hugr-py dependency to 0.13 ([#996](https://github.com/quantinuum/tket2/issues/996)) ([1bf4c70](https://github.com/quantinuum/tket2/commit/1bf4c70788693d357b3cb2dcdbe2c721951da2a5))
* Rename `tket2.*` HUGR extensions to `tket.*` ([#988](https://github.com/quantinuum/tket2/issues/988)) ([c5279c5](https://github.com/quantinuum/tket2/commit/c5279c55c1287980ff18c0bfdf360f69be5f345f))
* Rename tket2* libs to tket* ([#987](https://github.com/quantinuum/tket2/issues/987)) ([450f06a](https://github.com/quantinuum/tket2/commit/450f06ae6b2d7472ad33418299479709e307919c))

## [0.11.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.11.0...tket2-py-v0.11.1) (2025-07-09)


### Features

* Add CustomPass label to badger_pass ([#932](https://github.com/quantinuum/tket2/issues/932)) ([9896524](https://github.com/quantinuum/tket2/commit/98965241c3ca60e1e52852ed85d7609c1d38f397))
* Support pytket encoding/decoding of barriers ([#919](https://github.com/quantinuum/tket2/issues/919))

## [0.11.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.10.0...tket2-py-v0.11.0) (2025-06-16)

### Features

* Add V and Vdg to quantum extension. ([#889](https://github.com/quantinuum/tket2/issues/889)) ([a8f1569](https://github.com/quantinuum/tket2/commit/a8f156930ff3afc35af15a2afdd24fc65c8409b5))


### Code Refactoring

* More flexible pytket encoding ([#849](https://github.com/quantinuum/tket2/issues/849)) ([1895f68](https://github.com/quantinuum/tket2/commit/1895f6845efb6040cca45e81d1c71b4579e791b6))

## [0.10.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.9.1...tket2-py-v0.10.0) (2025-05-22)


### ⚠ BREAKING CHANGES

* In the `tket2.bool` extension, `BoolOp::bool_to_sum` / `BoolOp::sum_to_bool` renamed to `BoolOp::read` / `BoolOp::make_opaque` `Tk2Op::MeasureFree` now returns a `tket2.bool`

### Features

* Update `tket2-exts` dependency with breaking `tket2.bool` extension changes. ([b1cd078](https://github.com/quantinuum/tket2/commit/b1cd0784cf34146a778b21a98a8ab26c822b34cf))

## [0.9.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.9.0...tket2-py-v0.9.1) (2025-05-19)


### Bug Fixes

* update exts and eccs dependencies ([aae0558](https://github.com/quantinuum/tket2/commit/aae055870d51a554652400bbed04668b1c888e20))

## [0.9.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.8.1...tket2-py-v0.9.0) (2025-05-16)


### ⚠ BREAKING CHANGES

* bump to hugr 0.20

### Features

* **tket2-py:** Expose new modules in python ([#860](https://github.com/quantinuum/tket2/issues/860)) ([4bfb3ff](https://github.com/quantinuum/tket2/commit/4bfb3ff26f42b8dbf8e3fe29b4d451201cfbc989))


### Miscellaneous Chores

* Bump to hugr-0.20 ([#862](https://github.com/quantinuum/tket2/issues/862)) ([652a7d0](https://github.com/quantinuum/tket2/commit/652a7d0b039bca62407f16f7548204e97f92ef5a))

## [0.8.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.8.0...tket2-py-v0.8.1) (2025-03-18)


### Features

* Update `tket2-exts` extension to `0.6.0`
  ([e358bb1](https://github.com/quantinuum/tket2/commit/e358bb1a1641153cd718995da09888f98f0ffe35))
* Loosen pytket dependency to allow >=1.34

## [0.8.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.7.0...tket2-py-v0.8.0) (2025-03-17)


### ⚠ BREAKING CHANGES

* pre-envelope `Circuit` serialisation functions are deprecated.

### Features

* Add methods to en/decode from bytes ([#807](https://github.com/quantinuum/tket2/issues/807)) ([3e76fd2](https://github.com/quantinuum/tket2/commit/3e76fd2fb981889542b06cd218dfce9ad925cf29))
* Use hugr envelopes to store/load circuits ([#813](https://github.com/quantinuum/tket2/issues/813)) ([2940b2e](https://github.com/quantinuum/tket2/commit/2940b2e0c9b270259259690b83dbdf261543d26d))


### Miscellaneous Chores

* bump to hugr-rs 0.15 and hugr-py 0.11 ([#806](https://github.com/quantinuum/tket2/issues/806)) ([f3bfaae](https://github.com/quantinuum/tket2/commit/f3bfaae0b6e4b8fd934a62343317b85ccb8f96ee))

## [0.7.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.6.1...tket2-py-v0.7.0) (2025-03-06)


### ⚠ BREAKING CHANGES

* **tket2-py:** Adds `OrderInZones` to `tket2.qsystem.utils`

### Features

* **tket2-py:** bump tket2_exts constraint ([#794](https://github.com/quantinuum/tket2/issues/794)) ([520e950](https://github.com/quantinuum/tket2/commit/520e9505630d1bc166472fb1af1095bb39e8b414))

## [0.6.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.6.0...tket2-py-v0.6.1) (2025-02-21)


### Features

* add a `tket2.qsystem.utils` extension with `GetCurrentShot` ([#772](https://github.com/quantinuum/tket2/issues/772)) ([175a02d](https://github.com/quantinuum/tket2/commit/175a02da2ce8a0065c265cdae7518c1b5284cff3)), closes [#767](https://github.com/quantinuum/tket2/issues/767)


### Bug Fixes

* include RNG extension in tket2-py, bump tket2-exts constraint ([#781](https://github.com/quantinuum/tket2/issues/781)) ([9eb8897](https://github.com/quantinuum/tket2/commit/9eb8897fe5eed96070700b2f43461da1fd1346ee))

## [0.6.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.5.1...tket2-py-v0.6.0) (2024-12-16)


### ⚠ BREAKING CHANGES

* Updated `hugr-rs` to `0.14.0` / `hugr-py 0.10.0`. Extension are now defined in `hugr 0.14`-compatible format.

### Features

* update measurement and alloc operations ([#702](https://github.com/quantinuum/tket2/issues/702)) ([a7a0201](https://github.com/quantinuum/tket2/commit/a7a020116f42bfeb89c356d08816a2f3ce1b5226))


### Miscellaneous Chores

* Update to next version of hugr ([#720](https://github.com/quantinuum/tket2/issues/720)) ([4a3a5a5](https://github.com/quantinuum/tket2/commit/4a3a5a5e38252d4ee709e7e97bb5a1e90bd9fff4))


## [0.5.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.5.0...tket2-py-v0.5.1) (2024-11-29)


### Bug Fixes

* Remove use of removed auto_rebase_pass ([#708](https://github.com/quantinuum/tket2/issues/708)) ([ea122a7](https://github.com/quantinuum/tket2/commit/ea122a76444da0e94d745b02e2b475719cfd7bf7))

## [0.5.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.4.1...tket2-py-v0.5.0) (2024-11-11)


### ⚠ BREAKING CHANGES

* Removed `load_guppy_*` methods. Use Circuit::load_function_reader instead.

### Features

* move extensions to auxillary package ([#682](https://github.com/quantinuum/tket2/issues/682)) ([dd78f9a](https://github.com/quantinuum/tket2/commit/dd78f9a571905407bcf74131a54f4aefbca740c4))
* Track circuit extensions and read/write packages ([#680](https://github.com/quantinuum/tket2/issues/680)) ([5e87dd9](https://github.com/quantinuum/tket2/commit/5e87dd94f216a87f4d27dee44d178578d40e7ace))


## [0.4.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.4.0...tket2-py-v0.4.1) (2024-10-10)


### Features

* Add `tket2.rotation.from_halfturns_unchecked` op ([#640](https://github.com/quantinuum/tket2/issues/640)) ([86ffe64](https://github.com/quantinuum/tket2/commit/86ffe64fa455fc19a60999a7edb71803d615d77b))

## [0.4.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.3.0...tket2-py-v0.4.0) (2024-10-01)


### ⚠ BREAKING CHANGES

* "tket2.angle" extension replaced with "tket2.rotation" extension with rotation type and simplified set of operations.

### Features

* `BadgerOptimiser.load_precompiled`, `BadgerOptimiser.compile_eccs` and `passes.badger_pass` now take an optional `cost_fn` parameter to specify the cost function to minimise. Supported values are `'cx'` (default behaviour) and `'rz'`. ([83ebfcb](https://github.com/quantinuum/tket2/commit/83ebfcb9156fb5516f877155939062d11c7196d5))
* simplify angle extension in to a half turns rotation type ([#611](https://github.com/quantinuum/tket2/issues/611)) ([0723937](https://github.com/quantinuum/tket2/commit/0723937a8aed69302359fbd2383a01a77adc6b36))


### Bug Fixes

* Support hugr packages, fix the notebooks ([#622](https://github.com/quantinuum/tket2/issues/622)) ([1cf9dcb](https://github.com/quantinuum/tket2/commit/1cf9dcb7ba80dd236916bcf86a1fa0f5459fd349))


## [0.3.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.2.1...tket2-py-v0.3.0) (2024-09-04)


### ⚠ BREAKING CHANGES

* quantum extension renamed from "quantum.tket2" to "tket2.quantum"
* Parametric Tk2Ops take angle type rather than float.
* Rename lazy extension to "hseries" and add hseries ops.
* `Tk2Op::AngleAdd` removed, use `fadd` from the hugr float ops standard extension
* Updated to latest hugr version (`hugr-rs 0.12` / `hugr-py 0.8`). ECCs need to be recompiled.

### Features

* HSeries ops ([#573](https://github.com/quantinuum/tket2/issues/573)) ([e6acc88](https://github.com/quantinuum/tket2/commit/e6acc881d7ab67c584e6622d387bf2025209f8b8))
* remove Tk2Op::AngleAdd ([#567](https://github.com/quantinuum/tket2/issues/567)) ([42cc82f](https://github.com/quantinuum/tket2/commit/42cc82f0ee6e77dffb2f55c53613a7c4c8687824))
* replace f64 with angle type for tk2 ops ([#578](https://github.com/quantinuum/tket2/issues/578)) ([d14631f](https://github.com/quantinuum/tket2/commit/d14631f762f7ad2cf7db65e66b11cc38a54966ef))
* Update rust hugr dependency to `0.12.0`, and python hugr to `0.8.0` ([#568](https://github.com/quantinuum/tket2/issues/568)) ([258a7c5](https://github.com/quantinuum/tket2/commit/258a7c5ec25ee2665c524a174704944f0c19729e))


### Bug Fixes

* quantum extension name wrong way round ([#582](https://github.com/quantinuum/tket2/issues/582)) ([06a6838](https://github.com/quantinuum/tket2/commit/06a68386ce6d0b1376ed369da2a42d3b3eaa056a))

## [0.2.1](https://github.com/quantinuum/tket2/compare/tket2-py-v0.2.0...tket2-py-v0.2.1) (2024-08-14)

### ⚠ BREAKING CHANGES

* increased minimum required version of hugr to 0.7.0

### Documentation

* Add tket2-py module docstring ([#539](https://github.com/quantinuum/tket2/issues/539)) ([8ef7a57](https://github.com/quantinuum/tket2/commit/8ef7a5736294cf462b0694c235f0d10316c68f68))

## [0.2.0](https://github.com/quantinuum/tket2/compare/tket2-py-v0.1.0...tket2-py-v0.2.0) (2024-08-01)


### ⚠ BREAKING CHANGES

* `.rwr` ECC files generated with older versions are no longer supported. Please recompile them, or compress the file with `zstd`.

### Features

* Add timeout options and stats to Badger ([#496](https://github.com/quantinuum/tket2/issues/496)) ([32a9885](https://github.com/quantinuum/tket2/commit/32a98853db1415c6934fa26c966cb430c74023aa))
* Compress binary ECCs using zlib ([#498](https://github.com/quantinuum/tket2/issues/498)) ([d9a713c](https://github.com/quantinuum/tket2/commit/d9a713c068077dfea8c12301c1973575ddc8ca2c))
* Expose advanced Badger timeout options to tket2-py ([#506](https://github.com/quantinuum/tket2/issues/506)) ([fe7d40e](https://github.com/quantinuum/tket2/commit/fe7d40e4a176375c22fae5e52d42aeaf9ab38d33))
* Move the compiled eccs to a separate package ([#517](https://github.com/quantinuum/tket2/issues/517)) ([7247cc6](https://github.com/quantinuum/tket2/commit/7247cc65f4c4e679fd5b680d1e53e630f06d94a1))


### Bug Fixes

* Recompile ecc sets after [#441](https://github.com/quantinuum/tket2/issues/441) ([#484](https://github.com/quantinuum/tket2/issues/484)) ([1122fa4](https://github.com/quantinuum/tket2/commit/1122fa453790fe5fd3433d4e028fb327949d9619))


### Miscellaneous Chores

* bump hugr version to 0.10.0 ([#508](https://github.com/quantinuum/tket2/issues/508)) ([eca258b](https://github.com/quantinuum/tket2/commit/eca258bfbef5fcd82a0d3b3d70cb736e275b3487))

## [0.1.0a4](https://github.com/quantinuum/tket2/compare/tket2-py-v0.1.0...tket2-py-v0.1.0a4) (2024-07-15)


### Bug Fixes

* Recompile ecc sets after [#441](https://github.com/quantinuum/tket2/issues/441) ([#484](https://github.com/quantinuum/tket2/issues/484)) ([1122fa4](https://github.com/quantinuum/tket2/commit/1122fa453790fe5fd3433d4e028fb327949d9619))

## [0.1.0a3](https://github.com/quantinuum/tket2/compare/tket2-py-v0.1.0...tket2-py-v0.1.0a3) (2024-07-12)


### Documentation

* simplify the getting started notebook ([#466](https://github.com/quantinuum/tket2/issues/466)) ([10639b9](https://github.com/quantinuum/tket2/commit/10639b9ea6766cf9a91deb556dc1852881c7cd96))

## [0.1.0a2](https://github.com/quantinuum/tket2/compare/tket2-py-v0.0.0a1...tket2-py-v0.1.0a2) (2024-07-10)


### ⚠ BREAKING CHANGES

* `Dfg` binding removed, replaced with builder from `hugr` package.

### Features

* Add `lhs` and `rhs` bindings to `Rule` ([#440](https://github.com/quantinuum/tket2/issues/440)) ([49b1c89](https://github.com/quantinuum/tket2/commit/49b1c89aeeaff73c6ebf66d3f81dddc4ad81f7bb))
* get pauli propagation + examples working with new hugr builder [#465](https://github.com/quantinuum/tket2/issues/465) ([cab0d87](https://github.com/quantinuum/tket2/commit/cab0d8750c5bc92c4f1284e07146add371db233f))

## 0.1.0a1 (2024-06-28)

Initial alpha release of the Python bindings for TKET2.

Includes compatibility with pytket circuits and guppy definitions, as well as
some basic circuit passes. See the included examples for more information.
