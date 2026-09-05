#!/usr/bin/env bash
# Build the SYS/BIOS kernel exerciser for the emulated C6455.
#
# The RTOS pieces are TI's, built by TI's tools: xdc.tools.configuro generates the SYS/BIOS
# configuration (and, because SYS/BIOS 6.42 ships no prebuilt C64x+ library, compiles one from
# TI's sources), and the NDK libraries come from TI's own ndk.mak build. This script only adds
# the application and the NIMU driver.
set -euo pipefail


here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$here/../../.." && pwd)
tp="$root/tests/build"

# On NixOS, TI's tools are generic-Linux binaries and the SYS/BIOS installer is 32-bit; the
# devenv FHS environment provides the loaders they need. Re-exec inside it when available.
# This is inert on distributions where the tools run natively.
if [ -z "${TI_FHS_ACTIVE:-}" ] && command -v ti-tools-fhs >/dev/null 2>&1; then
    export TI_FHS_ACTIVE=1
    exec ti-tools-fhs -c "cd '$here' && exec '$0'"
fi

xdc="$tp/xdctools_3_32_01_22_core"
bios="$tp/bios_6_42_03_35"
ndk="$tp/ndk_2_22_02_16"
cgt="$tp/ti-cgt-c6000_8.5.0.LTS"
target=ti.targets.elf.C64P
platform=ti.platforms.dsk6455

for dir in "$xdc" "$bios" "$cgt"; do
    if [ ! -d "$dir" ]; then
        echo "missing $dir - run tools/fetch_deps.sh first" >&2
        exit 1
    fi
done

cd "$here"
echo "== configuro"
"$xdc/xs" --xdcpath="$bios/packages" \
    xdc.tools.configuro -o build -t "$target" -p "$platform" -r release -c "$cgt" app.cfg

echo "== compile"
compile_flags=(
    -mv6400+ --abi=eabi -q --symdebug:none -O2
    -I"$bios/packages"
    -I"$xdc/packages"
    -I"$cgt/include"
    -I build
)
"$cgt/bin/cl6x" "${compile_flags[@]}" -c main.c -fr=build -fs=build

echo "== link"
"$cgt/bin/cl6x" -mv6400+ --abi=eabi -q -z \
    build/main.obj \
    build/linker.cmd \
    -i"$cgt/lib" -llibc.a \
    --heap_size=0x4000 --stack_size=0x2000 \
    -m build/bios_kernel.map -o build/bios_kernel.out

echo "built $here/build/bios_kernel.out"
