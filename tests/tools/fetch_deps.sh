#!/usr/bin/env bash
# Fetch everything the conformance suite builds with and reads that is not checked in, into
# build/, so the tests run from a fresh checkout: TI's XDCtools, NDK, SYS/BIOS and C6000 code
# generation tools, TI's DSPLIB for C64x+, the tic6x test vectors out of a GNU binutils
# source release, and TI's Intermediate Boot Loader source, whose host tools build boot EEPROM
# images. A component is skipped when its tree is already present, and downloaded
# archives are kept in build/.downloads for re-runs (delete that directory to reclaim the
# space - roughly 600 MB).
#
# The installers and the XDCtools launchers are generic-Linux binaries (the SYS/BIOS and DSPLIB
# ones are 32-bit): run this script inside an environment that provides the system loaders, e.g.
# the devenv `ti-tools-fhs` FHS environment on NixOS, or install libc6-i386 on Debian/Ubuntu.
# The scripts themselves do not arrange for that.
#
# TI never shipped EABI C64x+ NDK libraries in any NDK release (verified across every 2.x
# version); tools/build_ndk_libs.sh builds them from the fetched NDK tree as part of the
# firmware build.
#
# Usage: tools/fetch_deps.sh [target-dir]
#   target-dir defaults to <repo>/tests/build.
set -euo pipefail

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$here/../.." && pwd)
target=${1:-"$root/tests/build"}
cache="$target/.downloads"

base="https://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent"
# Each entry is `name|url`, and each `name` is a directory under build/. The four groups are the
# four ways a release has to be opened; the names say what each does to that directory.
#
# Zip archives that already contain a top-level directory named exactly `name`.
zip_archives=(
    "xdctools_3_32_01_22_core|$base/rtsc/3_32_01_22/exports/xdccore/xdctools_3_32_01_22_core_linux.zip"
    "ndk_2_22_02_16|$base/ndk/2_22_02_16/exports/ndk_2_22_02_16.zip"
)
# Installers that create `name` themselves, under the prefix they are given. Both take
# `--mode unattended --prefix <parent-of-name>`.
installers_under_prefix=(
    "bios_6_42_03_35|$base/bios/sysbios/6_42_03_35/exports/bios_setuplinux_6_42_03_35.bin"
    "ti-cgt-c6000_8.3.15|https://dr-download.ti.com/software-development/ide-configuration-compiler-or-debugger/MD-vqU2jj6ibH/8.3.15/ti_cgt_c6000_8.3.15_linux-x64_installer.bin"
)
# Installers that unpack straight into the prefix, so the prefix has to be `name` itself. TI
# packaged this one with a different installer toolkit from the two above: it takes
# `--mode silent`, rejects `--mode unattended` with a Tcl usage trace, and creates no directory
# of its own. DSPLIB 3.4.0.0 is the execution corpus - 23 tests/programs/dsplib_* specs compile
# its natural-C kernels out of packages/ti/dsplib/src/.
installers_into_prefix=(
    "dsplib-c64plus|https://software-dl.ti.com/sdoemb/sdoemb_public_sw/dsplib/3_4_0_0/exports/dsplib_c64Px_3_4_0_0_Linux.bin"
)
# `name|url|member`: one directory of a source tarball, unpacked to `name`. binutils ships the
# tic6x vectors tests/tools/decode_sweep.py sweeps inside its full source release, so only that
# directory is extracted. 2.44 is the release the tic6x opcode tables the skills cite are pinned
# to; its vector files are byte-identical in 2.45.
tarball_subtrees=(
    "binutils-tic6x-vectors|https://ftp.gnu.org/gnu/binutils/binutils-2.44.tar.xz|binutils-2.44/gas/testsuite/gas/tic6x"
)
# `name|repository|commit`: a git tree checked out at one commit. git.ti.com serves no snapshot
# archives, but it does serve a single commit by its hash. The IBL is TI's own C6472 boot EEPROM
# tooling - the ROM's boot parameter header, b2i2c and romparse - which tests/test_i2cboot.py
# builds and uses as the oracle for the ROM's I2C boot (tests/tools/ibl_tools.py).
git_checkouts=(
    "ibl|https://git.ti.com/git/keystone-rtos/ibl.git|ed3f0989d8cdc813b7ca1ed899c4d84b63de3a75"
)

for tool in curl tar python3; do
    command -v "$tool" >/dev/null || { echo "$tool is required" >&2; exit 1; }
done
mkdir -p "$cache"

fetch() { # url outfile
    local url=$1 out=$2
    if [ ! -f "$out" ]; then
        echo "== downloading $(basename "$out")"
        curl -fL --retry 3 -o "$out" "$url"
    else
        echo "== $(basename "$out") already downloaded"
    fi
}

present() { # name — true when the tree is there and not empty
    local name=$1
    [ -d "$target/$name" ] && [ -n "$(ls -A "$target/$name")" ]
}

require_loader() { # installer
    # The SYS/BIOS and DSPLIB installers are 32-bit binaries and die without the 32-bit loader,
    # the SYS/BIOS one without even a message. Byte 4 of an ELF header is its class, 1 for
    # 32-bit, so which installers need the loader is read off the file rather than listed.
    [ "$(od -An -tu1 -j4 -N1 "$1" | tr -d ' ')" = 1 ] || return 0
    [ -e /lib/ld-linux.so.2 ] && return 0
    echo "error: $(basename "$1") is a 32-bit binary and /lib/ld-linux.so.2 is missing." >&2
    echo "  Ubuntu/Debian: sudo apt install libc6-i386" >&2
    echo "  NixOS: run this script inside an FHS environment that provides the loaders" >&2
    exit 1
}

unpack_zip() { # name url
    local name=$1 url=$2
    if present "$name"; then
        echo "== $name already present, skipping"
        return
    fi
    fetch "$url" "$cache/$name.zip"
    echo "== unpacking $name"
    python3 -m zipfile -e "$cache/$name.zip" "$target"
    # python's zipfile does not restore the executable bit. XDCtools ships xdc, xs and its
    # bin/, gmake/ and bundled JRE as executables, which the firmware build invokes.
    if [ "$name" = "xdctools_3_32_01_22_core" ]; then
        chmod +x "$target/$name/xdc" "$target/$name/xs" "$target/$name/xs.x86_64U"
        chmod -R +x "$target/$name/bin" "$target/$name/gmake" "$target/$name/jre/bin"
        # xs probes the OS with /bin/uname, which does not exist on NixOS.
        sed -i 's|`/bin/uname`|`uname`|' "$target/$name/xs"
    fi
    if ! present "$name"; then
        echo "error: archive did not extract to $target/$name" >&2
        exit 1
    fi
}

install_under_prefix() { # name url
    local name=$1 url=$2
    if present "$name"; then
        echo "== $name already present, skipping"
        return
    fi
    fetch "$url" "$cache/$name.bin"
    echo "== installing $name"
    chmod +x "$cache/$name.bin"
    require_loader "$cache/$name.bin"
    (cd "$cache" && "./$name.bin" --mode unattended --prefix "$target")
    if ! present "$name"; then
        echo "error: installer did not produce $target/$name" >&2
        exit 1
    fi
}

install_into_prefix() { # name url
    local name=$1 url=$2
    if present "$name"; then
        echo "== $name already present, skipping"
        return
    fi
    fetch "$url" "$cache/$name.bin"
    echo "== installing $name"
    chmod +x "$cache/$name.bin"
    require_loader "$cache/$name.bin"
    (cd "$cache" && "./$name.bin" --mode silent --prefix "$target/$name")
    if ! present "$name"; then
        echo "error: installer did not populate $target/$name" >&2
        exit 1
    fi
}

unpack_subtree() { # name url member
    local name=$1 url=$2 member=$3
    if present "$name"; then
        echo "== $name already present, skipping"
        return
    fi
    fetch "$url" "$cache/$name.tar.xz"
    echo "== unpacking $member"
    mkdir -p "$target/$name"
    # Strip every leading component of `member` so its files land directly in `name`, which is
    # what the consumers reference: the upstream path is an implementation detail of the release.
    tar -xf "$cache/$name.tar.xz" -C "$target/$name" \
        --strip-components "$(awk -F/ '{print NF}' <<<"$member")" "$member"
    if ! present "$name"; then
        echo "error: tarball did not yield $member" >&2
        exit 1
    fi
}

checkout() { # name repository commit
    local name=$1 repository=$2 commit=$3
    command -v git >/dev/null || { echo "git is required for $name" >&2; exit 1; }
    # A tree counts only when it is the pinned commit: anything else - including what an
    # interrupted fetch by an older version of this script left behind - is fetched again.
    if [ "$(git -C "$target/$name" rev-parse HEAD 2>/dev/null)" = "$commit" ]; then
        echo "== $name already present, skipping"
        return
    fi
    echo "== fetching $name at $commit"
    # Check out into a staging directory and publish it only once it is the commit, so a fetch
    # that fails leaves nothing that looks like the tree.
    local staging="$target/.$name.partial"
    rm -rf "$staging"
    git init -q "$staging"
    git -C "$staging" fetch -q --depth 1 "$repository" "$commit"
    git -C "$staging" checkout -q FETCH_HEAD
    if [ "$(git -C "$staging" rev-parse HEAD)" != "$commit" ]; then
        echo "error: $name is not at $commit" >&2
        exit 1
    fi
    rm -rf "$target/$name"
    mv "$staging" "$target/$name"
}

for entry in "${zip_archives[@]}"; do
    unpack_zip "${entry%%|*}" "${entry#*|}"
done
for entry in "${installers_under_prefix[@]}"; do
    install_under_prefix "${entry%%|*}" "${entry#*|}"
done
for entry in "${installers_into_prefix[@]}"; do
    install_into_prefix "${entry%%|*}" "${entry#*|}"
done
for entry in "${tarball_subtrees[@]}"; do
    rest=${entry#*|}
    unpack_subtree "${entry%%|*}" "${rest%%|*}" "${rest#*|}"
done
for entry in "${git_checkouts[@]}"; do
    rest=${entry#*|}
    checkout "${entry%%|*}" "${rest%%|*}" "${rest#*|}"
done

echo
echo "Test dependencies ready in $target:"
for name in "${zip_archives[@]%%|*}" "${installers_under_prefix[@]%%|*}" \
            "${installers_into_prefix[@]%%|*}" "${tarball_subtrees[@]%%|*}" \
            "${git_checkouts[@]%%|*}"; do
    if present "$name"; then
        echo "  - $name"
    else
        echo "  - $name MISSING" >&2
        exit 1
    fi
done
echo "Firmware rebuilds: firmware/ndk_tcp/build.sh and firmware/bios_kernel/build.sh"
