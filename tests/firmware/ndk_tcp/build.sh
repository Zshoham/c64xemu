#!/usr/bin/env bash
# Build the NDK TCP echo firmware for the emulated C6455.
#
# The RTOS pieces are TI's, built by TI's tools: xdc.tools.configuro generates the SYS/BIOS
# configuration (and, because SYS/BIOS 6.42 ships no prebuilt C64x+ library, compiles one from
# TI's sources), and tools/build_ndk_libs.sh builds the EABI C64x+ NDK libraries (TI ships
# none in any release). This script only adds the application and the NIMU driver.
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
cgt="$tp/ti-cgt-c6000_8.3.15"
target=ti.targets.elf.C64P
platform=ti.platforms.dsk6455

for dir in "$xdc" "$bios" "$ndk" "$cgt"; do
    if [ ! -d "$dir" ]; then
        echo "missing $dir - run tools/fetch_deps.sh first" >&2
        exit 1
    fi
done

# configuro validates the NDK libraries the NDK config references, so the EABI C64x+ ones
# (which TI ships in no release) must exist before it runs. Idempotent.
"$root/tests/tools/build_ndk_libs.sh"

cd "$here"
echo "== configuro"
"$xdc/xs" --xdcpath="$bios/packages;$ndk/packages" \
    xdc.tools.configuro -o build -t "$target" -p "$platform" -r release -c "$cgt" app.cfg

echo "== compile"
compile_flags=(
    -mv6400+ --abi=eabi -q --symdebug:none -O2
    -I"$ndk/packages/ti/ndk/inc"
    -I"$ndk/packages/ti/ndk/inc/tools"
    -I"$ndk/packages"
    -I"$bios/packages"
    -I"$xdc/packages"
    -I"$cgt/include"
    -I build
)
"$cgt/bin/cl6x" "${compile_flags[@]}" -c main.c nimu_emac.c -fr=build -fs=build

echo "== link"
# The generated linker command file names the SYS/BIOS libraries; the NDK libraries are named
# here in dependency order: netctrl, then the tools/services, the stack, and the OS layer.
ndk_lib="$ndk/packages/ti/ndk"
"$cgt/bin/cl6x" -mv6400+ --abi=eabi -q -z \
    build/main.obj build/nimu_emac.obj \
    build/linker.cmd \
    "$ndk_lib/netctrl/lib/netctrl_min_ipv4.ae64P" \
    "$ndk_lib/tools/console/lib/console_min_ipv4.ae64P" \
    "$ndk_lib/tools/servers/lib/servers_min_ipv4.ae64P" \
    "$ndk_lib/nettools/lib/nettool_ipv4.ae64P" \
    "$ndk_lib/stack/lib/stk.ae64P" \
    "$ndk_lib/os/lib/os.ae64P" \
    "$ndk_lib/hal/timer_bios/lib/hal_timer_bios.ae64P" \
    "$ndk_lib/hal/userled_stub/lib/hal_userled_stub.ae64P" \
    -i"$cgt/lib" -llibc.a \
    --heap_size=0x4000 --stack_size=0x2000 \
    -m build/ndk_tcp.map -o build/ndk_tcp.out

echo "built $here/build/ndk_tcp.out"
