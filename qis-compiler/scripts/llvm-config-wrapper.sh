#!/bin/sh
# A drop-in replacement for llvm-config, used in manylinux_2_28 containers
# where the real llvm-config binary cannot execute due to glibc version
# mismatch (the official LLVM binaries are built on glibc 2.35, but
# manylinux_2_28 has glibc 2.28).
#
# The static libraries (.a) and headers from the LLVM tarball work fine
# regardless of glibc version -- only the llvm-config binary itself needs
# to run, so we replace it with this script.
#
# This script is designed to satisfy llvm-sys's build.rs queries.
# See https://gitlab.com/taricorp/llvm-sys.rs

PREFIX=/tmp/llvm
VERSION=21.1.8

for arg in "$@"; do
    case "$arg" in
        --version)
            echo "$VERSION"
            ;;
        --prefix)
            echo "$PREFIX"
            ;;
        --includedir)
            echo "$PREFIX/include"
            ;;
        --libdir)
            echo "$PREFIX/lib"
            ;;
        --cflags)
            echo "-I$PREFIX/include -D_GNU_SOURCE -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D__STDC_LIMIT_MACROS"
            ;;
        --cxxflags)
            echo "-I$PREFIX/include -std=c++17 -fno-exceptions -fno-rtti -D_GNU_SOURCE -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D__STDC_LIMIT_MACROS"
            ;;
        --ldflags)
            echo "-L$PREFIX/lib"
            ;;
        --system-libs)
            echo "-lrt -ldl -lpthread -lm -lz -ltinfo"
            ;;
        --libs)
            # Output -l flags for all LLVM static libraries
            result=""
            for f in "$PREFIX"/lib/libLLVM*.a; do
                name=$(basename "$f" .a)
                name=${name#lib}
                result="$result -l$name"
            done
            echo "$result"
            ;;
        --libnames)
            # Output library filenames
            result=""
            for f in "$PREFIX"/lib/libLLVM*.a; do
                result="$result $(basename "$f")"
            done
            echo "$result"
            ;;
        --libfiles)
            # Output full paths to library files
            result=""
            for f in "$PREFIX"/lib/libLLVM*.a; do
                result="$result $f"
            done
            echo "$result"
            ;;
        --components)
            result=""
            for f in "$PREFIX"/lib/libLLVM*.a; do
                name=$(basename "$f" .a)
                name=${name#libLLVM}
                result="$result $(echo "$name" | tr '[:upper:]' '[:lower:]')"
            done
            echo "$result"
            ;;
        --shared-mode)
            echo "static"
            ;;
        --build-mode)
            echo "Release"
            ;;
        --has-rtti)
            echo "NO"
            ;;
        --link-static|--link-shared)
            # Modifier flags, no output
            ;;
        -*)
            # Unknown flag, ignore
            ;;
        *)
            # Component names or other positional args, ignore
            ;;
    esac
done
