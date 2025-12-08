# Representation of quantum types in HUGR

TKET makes use of the
[HUGR](https://github.com/quantinuum/hugr/blob/main/specification/hugr.md)
IR to represent programs. Here we describe how quantum operations (and
the angles associated with some of them) are represented.

Besides a range of quantum operations (like Hadamard, CX, etc.)
that take and return `Qubit`, we note the following operations for
allocating/deallocating `Qubit`s:

```
qalloc: () -> Qubit
qfree: Qubit -> ()
```

`qalloc` allocates a fresh, 0 state Qubit - if none is available at
runtime it panics. `qfree` loses a handle to a Qubit (may be reallocated
in future). The point at which an allocated qubit is reset may be
target/compiler specific.

Note there are also `measurez: Qubit -> (i1, Qubit)` and on supported
targets `reset: Qubit -> Qubit` operations to measure or reset a qubit
without losing a handle to it.

## Dynamic vs static allocation

With these operations the programmer/front-end can request dynamic qubit
allocation, and the compiler can add/remove/move these operations to use
more or fewer qubits. In some use cases, that may not be desirable, and
we may instead want to guarantee only a certain number of qubits are
used by the program. For this purpose TKET places additional
constraints on the HUGR that are in line with TKET1 backwards
compatibility:

1. The `main` function takes one `Array<N, Qubit>`
input and has one output of the same type (the same statically known
size).
2. All Operations that have a `Signature` involving `Qubit` have as
 many `Qubit` input wires as output.


With these constraints, we can treat all `Qubit` operations as returning all qubits they take
in. The implicit bijection from input `Qubit` to output allows register
allocation for all `Qubit` wires.
If further the program does not contain any `qalloc` or `qfree`
operations we can state the program only uses `N` qubits.

## Angles

The "angle" extension defines a specialized `angle<N>` type which is used
to express parameters of rotation gates. The type is parametrized by the
_log-denominator_, which is an integer $N \in [0, 53]$; angles with
log-denominator $N$ are multiples of $2 \pi / 2^N$, where the multiplier is an
unsigned `int<N>` in the range $[0, 2^N]$. The maximum log-denominator $53$
effectively gives the resolution of a `float64` value; but note that unlike
`float64` all angle values are equatable and hashable; and two `angle<N>` that
differ by a multiple of $2 \pi$ are _equal_.

The following operations are defined:

| Name           | Inputs     | Outputs    | Meaning |
| -------------- | ---------- | ---------- | ------- |
| `aconst<N, x>` | none       | `angle<N>` | const node producing angle $2 \pi x / 2^N$ (where $0 \leq x \lt 2^N$) |
| `atrunc<M,N>`  | `angle<M>` | `angle<N>` | round `angle<M>` to `angle<N>`, where $M \geq N$, rounding down in $[0, 2\pi)$ if necessary |
| `aconvert<M,N>`  | `angle<M>` | `Sum(angle<N>, ErrorType)` | convert `angle<M>` to `angle<N>`, returning an error if $M \gt N$ and exact conversion is impossible |
| `aadd<M,N>`    | `angle<M>`, `angle<N>` | `angle<max(M,N)>` | add two angles |
| `asub<M,N>`    | `angle<M>`, `angle<N>` | `angle<max(M,N)>` | subtract the second angle from the first |
| `aneg<N>`      | `angle<N>` | `angle<N>` | negate an angle |
