# Changelog


## [0.16.0](https://github.com/quantinuum/tket2/compare/tket-v0.15.0...tket-v0.16.0) - 2025-10-20

### Bug Fixes

- *(pytket-encoder)* Correctly encode I/O nodes in nested containers ([#1117](https://github.com/quantinuum/tket2/pull/1117))
- Ensure decoded circuits have the required signature ([#1123](https://github.com/quantinuum/tket2/pull/1123))
- Avoid loops when encoding recursive fns ([#1143](https://github.com/quantinuum/tket2/pull/1143))
- Badger performance improvements ([#1139](https://github.com/quantinuum/tket2/pull/1139))

### New Features

- [**breaking**] SerialCircuit::decode_inplace and explicit option structs ([#1120](https://github.com/quantinuum/tket2/pull/1120))
- Decode pytket circs into DFGs ([#1121](https://github.com/quantinuum/tket2/pull/1121))
- Decode CircBoxes as DFG nodes ([#1122](https://github.com/quantinuum/tket2/pull/1122))
- *(pytket-decoder)* [**breaking**] Allow specifying qubit/bit reuse ([#1127](https://github.com/quantinuum/tket2/pull/1127))
- pull out unpack functionality from barrier handling ([#1144](https://github.com/quantinuum/tket2/pull/1144))
- Deal with non-local edges in pytket encoder ([#1145](https://github.com/quantinuum/tket2/pull/1145))
- Definition of extension ops for modifiers and global phases ([#1137](https://github.com/quantinuum/tket2/pull/1137))
- modifier resolver for various `OpTypes` ([#1162](https://github.com/quantinuum/tket2/pull/1162))
- Modifier for TketOps and global phases ([#1168](https://github.com/quantinuum/tket2/pull/1168))

### Refactor

- Create backtracking optimiser, refactor badger to use it ([#977](https://github.com/quantinuum/tket2/pull/977))

## [0.15.0](https://github.com/quantinuum/tket2/compare/tket-v0.14.0...tket-v0.15.0) - 2025-09-15

### Bug Fixes

- [**breaking**] Fix rotation -> float param type conversion ([#1061](https://github.com/quantinuum/tket2/pull/1061))
- Pytket barrier operations not being decoded ([#1069](https://github.com/quantinuum/tket2/pull/1069))
- Always load parameter expressions as half turns in the decoder ([#1083](https://github.com/quantinuum/tket2/pull/1083))

### New Features

- Capture pytket's output permutation explicitly in the hugr connectivity ([#1075](https://github.com/quantinuum/tket2/pull/1075))
- Add ResourceScope ([#1052](https://github.com/quantinuum/tket2/pull/1052))
- [**breaking**] Remove unnecessary Arc from PytketDecoder method ([#1114](https://github.com/quantinuum/tket2/pull/1114))
- [**breaking**] Remove deprecated definitions ([#1113](https://github.com/quantinuum/tket2/pull/1113))

## [0.14.0](https://github.com/quantinuum/tket2/compare/tket-v0.13.2...tket-v0.14.0) - 2025-08-18

### New Features

- [**breaking**] Allow PytketTypeTranslators to translate nested types ([#1038](https://github.com/quantinuum/tket2/pull/1038))
- Define a wire tracker for the new pytket decoder ([#1036](https://github.com/quantinuum/tket2/pull/1036))
- [**breaking**] Reworked pytket decoder framework ([#1030](https://github.com/quantinuum/tket2/pull/1030))
- [**breaking**] Use qsystem encoder/decoders in tket-py ([#1041](https://github.com/quantinuum/tket2/pull/1041))
- [**breaking**] Avoid eagerly cloning SerialCircuits when decoding from pytket ([#1048](https://github.com/quantinuum/tket2/pull/1048))

### Refactor

- [**breaking**] Rename tk2 encoder names to tket ([#1037](https://github.com/quantinuum/tket2/pull/1037))

## [0.13.2](https://github.com/quantinuum/tket2/compare/tket-v0.13.1...tket-v0.13.2) - 2025-08-08

### New Features

- *(tket-py)* Create BadgerOptimiser from arbitrary Rewriters ([#1022](https://github.com/quantinuum/tket2/pull/1022))

## [0.13.1](https://github.com/quantinuum/tket2/compare/tket-v0.13.0...tket-v0.13.1) - 2025-07-30

### Documentation

- Update README badges ([#1004](https://github.com/quantinuum/tket2/pull/1004))
## [0.13.0](https://github.com/quantinuum/tket2/compare/tket2-v0.12.3...tket-v0.13.0) - 2025-07-25

### New Features

- [**breaking] Rename tket2.* HUGR extensions to tket.* ([#988](https://github.com/quantinuum/tket2/pull/988))
- [**breaking] Rename tket2* libs to tket* ([#987](https://github.com/quantinuum/tket2/pull/987))
- [**breaking**] Update to `hugr 0.21` ([#965](https://github.com/quantinuum/tket2/pull/965))
- Add guppy extension with drop operation ([#962](https://github.com/quantinuum/tket2/pull/962))
- [**breaking**] Split the pytket extension encoder trait ([#970](https://github.com/quantinuum/tket2/pull/970))

## [0.12.3](https://github.com/quantinuum/tket2/compare/tket2-v0.12.2...tket2-v0.12.3) - 2025-07-08

### New Features

- Add llvm codegen for `tket2.bool` ([#950](https://github.com/quantinuum/tket2/pull/950))

## [0.12.2](https://github.com/quantinuum/tket2/compare/tket2-v0.12.1...tket2-v0.12.2) - 2025-06-30

### Refactor

- *(hseries)* use array unpack operation ([#913](https://github.com/quantinuum/tket2/pull/913))

## [0.12.1](https://github.com/quantinuum/tket2/compare/tket2-v0.12.0...tket2-v0.12.1) - 2025-06-18

### New Features

- *(pytket encoding)* Support prelude barriers ([#919](https://github.com/quantinuum/tket2/pull/919))

## [0.12.0](https://github.com/quantinuum/tket2/compare/tket2-v0.11.1...tket2-v0.12.0) - 2025-06-16

### New Features

- *(pytket-encoding)* Bit initialization and bool operations ([#907](https://github.com/quantinuum/tket2/pull/907))
- *(pytket-encoding)* Pytket encode dfgs and function calls ([#908](https://github.com/quantinuum/tket2/pull/908))
- Add llvm lowering for debug extension ([#900](https://github.com/quantinuum/tket2/pull/900))

### Refactor

- [**breaking**] More flexible pytket encoding ([#849](https://github.com/quantinuum/tket2/pull/849))

## [0.11.1](https://github.com/quantinuum/tket2/compare/tket2-v0.11.0...tket2-v0.11.1) - 2025-06-03

### New Features

- Add V and Vdg to quantum extension. ([#889](https://github.com/quantinuum/tket2/pull/889))
- *(tket2-hseries)* LLVM codegen for extensions ([#898](https://github.com/quantinuum/tket2/pull/898))

## [0.11.0](https://github.com/quantinuum/tket2/compare/tket2-v0.10.0...tket2-v0.11.0) - 2025-05-22

### ⚠ BREAKING CHANGES
- Tk2Op::MeasureFree now returns a tket2.bool

### New Features

- [**breaking**] Add `ReplaceBoolPass` ([#854](https://github.com/quantinuum/tket2/pull/854))

### Refactor

- Use black_box from standard library. ([#878](https://github.com/quantinuum/tket2/pull/878))

## [0.10.0](https://github.com/quantinuum/tket2/compare/tket2-v0.9.0...tket2-v0.10.0) - 2025-05-16

### New Features

- [**breaking**] bump msrv to 1.85 ([#868](https://github.com/quantinuum/tket2/pull/868))

## [0.9.0](https://github.com/quantinuum/tket2/compare/tket2-v0.8.0...tket2-v0.9.0) - 2025-05-06

### New Features

- Add `tket2.bool` extension ([#823](https://github.com/quantinuum/tket2/pull/823))
- Add llvm codegen for `tket2.rotation` extension ([#851](https://github.com/quantinuum/tket2/pull/851))
- Add debug extension with state result op ([#843](https://github.com/quantinuum/tket2/pull/843))

### Refactor

- Better error message on allocation failure. ([#827](https://github.com/quantinuum/tket2/pull/827))
- [**breaking**] Remove node parameter from Circuit ([#824](https://github.com/quantinuum/tket2/pull/824))

## [0.8.0](https://github.com/quantinuum/tket2/compare/tket2-v0.7.3...tket2-v0.8.0) - 2025-03-17

### New Features

- [**breaking**] Use hugr envelopes to store/load circuits ([#813](https://github.com/quantinuum/tket2/pull/813))

## [0.7.1](https://github.com/quantinuum/tket2/compare/tket2-v0.7.0...tket2-v0.7.1) - 2025-01-10

### Bug Fixes

- remove unicode pi symbols in rotation extension (#743)

## [0.7.0](https://github.com/quantinuum/tket2/compare/tket2-v0.6.0...tket2-v0.7.0) - 2024-12-16

### ⚠ BREAKING CHANGES

- Removed `load_guppy_*` methods. Use `Circuit::load_function_reader` instead.
- Extension definitions and registries now use `Arc`s for sharing

### New Features

- [**breaking**] Track circuit extensions and read/write packages (#680)
- [**breaking**] update measurement and alloc operations (#702)

### Refactor

- [**breaking**] update to hugr 0.14 (#700)

## [0.6.0](https://github.com/quantinuum/tket2/compare/tket2-v0.5.0...tket2-v0.6.0) - 2024-10-15

### New Features

- *(badger)* `cx` and `rz` const functions and strategies for `LexicographicCostFunction` ([#625](https://github.com/quantinuum/tket2/pull/625))
- Add `tket2.rotation.from_halfturns_unchecked` op ([#640](https://github.com/quantinuum/tket2/pull/640))
- [**breaking**] update to hugr 0.13.0 ([#645](https://github.com/quantinuum/tket2/pull/645))
- Decode pytket op parameters ([#644](https://github.com/quantinuum/tket2/pull/644))
- re-export hugr crate ([#652](https://github.com/quantinuum/tket2/pull/652))
- Extract pytket parameters to input wires ([#661](https://github.com/quantinuum/tket2/pull/661))

### Refactor

- [**breaking**] Remove deprecated exports ([#662](https://github.com/quantinuum/tket2/pull/662))

## [0.5.0](https://github.com/quantinuum/tket2/compare/tket2-v0.4.0...tket2-v0.5.0) - 2024-09-30

### Bug Fixes

- Support hugr packages, fix the notebooks ([#622](https://github.com/quantinuum/tket2/pull/622))

### New Features

- Add an explicit struct for the tket2 sympy op ([#616](https://github.com/quantinuum/tket2/pull/616))
- Support encoding float and sympy ops ([#618](https://github.com/quantinuum/tket2/pull/618))

## [0.4.0](https://github.com/quantinuum/tket2/compare/tket2-v0.3.0...tket2-v0.4.0) - 2024-09-16

### Bug Fixes

- angle type docstring to say 2pi ([#607](https://github.com/quantinuum/tket2/pull/607))
- Fix broken ConstAngle::TAU ([#609](https://github.com/quantinuum/tket2/pull/609))

### New Features

- [**breaking**] simplify angle extension in to a half turns rotation type ([#611](https://github.com/quantinuum/tket2/pull/611))

## [0.3.0](https://github.com/quantinuum/tket2/compare/tket2-v0.2.0...tket2-v0.3.0) - 2024-09-09

### Bug Fixes

- extension ops checking against incorrect name ([#593](https://github.com/quantinuum/tket2/pull/593))
- [**breaking**] remove TryFrom for extension ops use `cast` ([#592](https://github.com/quantinuum/tket2/pull/592))
- don't load angle extensions on to quantum ([#597](https://github.com/quantinuum/tket2/pull/597))

### New Features

- [**breaking**] move angle types + and ops to new "tket2.angle" extension ([#591](https://github.com/quantinuum/tket2/pull/591))
- dataflow builder methods for angle ops ([#596](https://github.com/quantinuum/tket2/pull/596))
- lowering tk2ops -> hseriesops ([#579](https://github.com/quantinuum/tket2/pull/579))

## [0.2.0](https://github.com/quantinuum/tket2/compare/tket2-v0.1.1...tket2-v0.2.0) - 2024-09-04

### Bug Fixes
- [**breaking**] quantum extension name wrong way round ([#582](https://github.com/quantinuum/tket2/pull/582))

### New Features
- Extend Command::optype lifetime ([#562](https://github.com/quantinuum/tket2/pull/562))
- [**breaking**] Update rust hugr dependency to `0.12.0`, and python hugr to `0.8.0` ([#568](https://github.com/quantinuum/tket2/pull/568))
- [**breaking**] remove Tk2Op::AngleAdd ([#567](https://github.com/quantinuum/tket2/pull/567))
- [**breaking**] angle type no longer parametric. ([#577](https://github.com/quantinuum/tket2/pull/577))
- [**breaking**] HSeries ops ([#573](https://github.com/quantinuum/tket2/pull/573))
- [**breaking**] replace f64 with angle type for tk2 ops ([#578](https://github.com/quantinuum/tket2/pull/578))
- more angle ops (construct, deconstruct, radians, equality) ([#581](https://github.com/quantinuum/tket2/pull/581))

## [0.1.1](https://github.com/quantinuum/tket2/compare/tket2-v0.1.0...tket2-v0.1.1) - 2024-08-15

### New Features
- Move parallel evaluation code to CircuitChunks ([#528](https://github.com/quantinuum/tket2/pull/528))


## [0.1.0](https://github.com/quantinuum/tket2/compare/tket2-v0.1.0-alpha.2...tket2-v0.1.0) - 2024-08-01

### Bug Fixes
- Single source of truth for circuit names, and better circuit errors ([#390](https://github.com/quantinuum/tket2/pull/390))
- Support non-DFG circuits ([#391](https://github.com/quantinuum/tket2/pull/391))
- Portmatching not matching const edges ([#444](https://github.com/quantinuum/tket2/pull/444))
- Pattern matcher discriminating on opaqueOp description ([#441](https://github.com/quantinuum/tket2/pull/441))
- `extract_dfg` inserting the output node with an invalid child order ([#442](https://github.com/quantinuum/tket2/pull/442))
- Recompile ecc sets after [#441](https://github.com/quantinuum/tket2/pull/441) ([#484](https://github.com/quantinuum/tket2/pull/484))

### Documentation
- Update tket2-py readme ([#431](https://github.com/quantinuum/tket2/pull/431))
- Better error reporting in portmatching ([#437](https://github.com/quantinuum/tket2/pull/437))
- Improved multi-threading docs for Badger ([#495](https://github.com/quantinuum/tket2/pull/495))

### New Features
- `Circuit::operations` ([#395](https://github.com/quantinuum/tket2/pull/395))
- tuple unpack rewrite ([#406](https://github.com/quantinuum/tket2/pull/406))
- guppy → pytket conversion ([#407](https://github.com/quantinuum/tket2/pull/407))
- Drop linear bits, improve pytket encoding/decoding ([#420](https://github.com/quantinuum/tket2/pull/420))
- *(py)* Allow using `Tk2Op`s in the builder ([#436](https://github.com/quantinuum/tket2/pull/436))
- Initial support for `TailLoop` as circuit parent ([#417](https://github.com/quantinuum/tket2/pull/417))
- Support tuple unpacking with multiple unpacks ([#470](https://github.com/quantinuum/tket2/pull/470))
- Partial tuple unpack ([#475](https://github.com/quantinuum/tket2/pull/475))
- [**breaking**] Compress binary ECCs using zlib ([#498](https://github.com/quantinuum/tket2/pull/498))
- Add timeout options and stats to Badger ([#496](https://github.com/quantinuum/tket2/pull/496))
- Expose advanced Badger timeout options to tket2-py ([#506](https://github.com/quantinuum/tket2/pull/506))

### Refactor
- [**breaking**] Simplify tket1 conversion errors ([#408](https://github.com/quantinuum/tket2/pull/408))
- Cleanup tket1 serialized op structures ([#419](https://github.com/quantinuum/tket2/pull/419))

### Testing
- Add coverage for Badger split circuit multi-threading ([#505](https://github.com/quantinuum/tket2/pull/505))

## [0.1.0-alpha.2](https://github.com/quantinuum/tket2/compare/tket2-v0.1.0-alpha.1...tket2-v0.1.0-alpha.2) - 2024-06-11

### Bug Fixes
- Commands iterator ignoring the hierarchy. ([#381](https://github.com/quantinuum/tket2/pull/381))

### New Features
- Replace `Circuit::num_gates` with `num_operations` ([#384](https://github.com/quantinuum/tket2/pull/384))
- Utilities for loading compiled guppy circuits ([#393](https://github.com/quantinuum/tket2/pull/393))

### Refactor
- [**breaking**] Replace Circuit trait with a struct ([#370](https://github.com/quantinuum/tket2/pull/370))
- [**breaking**] Rename `tket2::json` into `tket2::serialize::pytket` ([#392](https://github.com/quantinuum/tket2/pull/392))

## [0.1.0-alpha.1](https://github.com/quantinuum/tket2/releases/tag/tket2-v0.1.0-alpha.1) - 2024-05-24

Initial alpha release of the library
