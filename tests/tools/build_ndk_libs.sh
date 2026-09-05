#!/usr/bin/env bash
# Build the EABI C64x+ NDK libraries the firmware links, into the fetched NDK tree under
# build/. TI ships no EABI C64x+ NDK libraries in any release (checked across every
# 2.x version), so the missing ti.targets.elf.C64P target is added to ndk.mak/ndk.bld from a
# patch file and the libraries are built from source with TI's own XDC build engine.
#
# Idempotent: exits immediately when the libraries are already present. Requires the
# XDCtools, SYS/BIOS and NDK trees and the C6000 code generation tools in build/
# (tools/fetch_deps.sh). Run inside an environment that can execute the XDCtools
# launchers (devenv ti-tools-fhs on NixOS; firmware/*/build.sh arranges that).
set -euo pipefail

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$here/../.." && pwd)
tp="$root/tests/build"
xdc="$tp/xdctools_3_32_01_22_core"
bios="$tp/bios_6_42_03_35"
ndk="$tp/ndk_2_22_02_16"
cgt="$tp/ti-cgt-c6000_8.5.0.LTS"
probe="$ndk/packages/ti/ndk/stack/lib/stk.ae64P"

if [ -f "$probe" ]; then
    echo "== NDK EABI libraries already built, skipping"
    exit 0
fi
for dir in "$xdc" "$bios" "$ndk" "$cgt"; do
    if [ ! -d "$dir" ]; then
        echo "missing $dir - run tools/fetch_deps.sh first" >&2
        exit 1
    fi
done

# TI's 2.22 ndk.mak (and 2.25.01.11, same layout) predates the EABI C64x+ target: it neither
# declares the `ti.targets.elf.C64P` variable nor forwards it through XDCARGS, and ndk.bld's
# ccOpts map has no entry for it. Without this patch the XDC build configures zero targets.
if ! grep -qF 'ti.targets.elf.C64P="$(ti.targets.elf.C64P)"' "$ndk/ndk.mak"; then
    echo "== applying ndk EABI C64x+ patch"
    patch -p1 -d "$ndk" < "$root/tests/tools/patches/ndk-2.22-eabi-c64p.patch"
fi

make_cmd=
for candidate in gmake make; do
    if command -v "$candidate" >/dev/null 2>&1; then
        make_cmd=$candidate
        break
    fi
done
if [ -z "$make_cmd" ]; then
    echo "error: GNU make is required to build the NDK EABI libraries" >&2
    exit 1
fi

echo "== building NDK EABI (ae64P) libraries with $make_cmd"
(cd "$ndk" && "$make_cmd" -f ndk.mak \
    XDC_INSTALL_DIR="$xdc" \
    SYSBIOS_INSTALL_DIR="$bios" \
    ti.targets.elf.C64P="$cgt")
if [ ! -f "$probe" ]; then
    echo "error: the NDK EABI build did not produce $probe" >&2
    exit 1
fi
echo "built NDK EABI libraries: $(cd "$ndk/packages/ti/ndk" && ls -1 */lib/*.ae64P | wc -l) libraries"
