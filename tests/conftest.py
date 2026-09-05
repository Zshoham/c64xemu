from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from common import BIOS_IMAGE, NDK_IMAGE, ROOT, Case, locate_candidate, selected_case_ids
from test_programs import program_cases


@pytest.fixture(scope="session", autouse=True)
def test_dependencies() -> None:
    """Provision what the suites build with and read, and build the firmware images.

    None of it is checked in: build/ is provisioned with tests/tools/fetch_deps.sh when any
    tree is absent - the TI toolchains the firmware and target programs are built with,
    DSPLIB, whose kernels the dsplib_* program specs compile, and the binutils tic6x vectors
    the decoder differential sweeps - and the firmware images and the EABI C64x+ NDK
    libraries they link are then built, bios_kernel first, then ndk_tcp (which builds the NDK
    libraries via tools/build_ndk_libs.sh).
    """
    trees = [ROOT / "tests/build" / name for name in
             ("xdctools_3_32_01_22_core", "bios_6_42_03_35", "ndk_2_22_02_16",
              "ti-cgt-c6000_8.5.0.LTS", "dsplib-c64plus", "binutils-tic6x-vectors")]
    if not all(tree.is_dir() for tree in trees):
        print("provisioning build/ with tests/tools/fetch_deps.sh", file=sys.stderr)
        # The installers and XDCtools launchers are generic-Linux binaries; on NixOS they run
        # inside the devenv FHS environment, so invoke the script through it when present.
        command = [str(ROOT / "tests/tools/fetch_deps.sh")]
        env = os.environ.copy()
        fhs = shutil.which("ti-tools-fhs")
        if fhs and env.get("TI_FHS_ACTIVE") != "1":
            env["TI_FHS_ACTIVE"] = "1"
            command = [fhs, "-c", f"cd '{ROOT}' && exec tests/tools/fetch_deps.sh"]
        completed = subprocess.run(command, cwd=ROOT, timeout=3600, env=env)
        if completed.returncode != 0:
            pytest.fail(f"build provisioning failed (exit {completed.returncode})")
    missing = [image for image in (BIOS_IMAGE, NDK_IMAGE) if not image.is_file()]
    if not missing:
        return
    print(f"building firmware images: {', '.join(path.name for path in missing)}", file=sys.stderr)
    for script in ("tests/firmware/bios_kernel/build.sh", "tests/firmware/ndk_tcp/build.sh"):
        completed = subprocess.run([str(ROOT / script)], cwd=ROOT, timeout=1800)
        if completed.returncode != 0:
            pytest.fail(f"firmware build {script} failed (exit {completed.returncode})")
    for image in missing:
        if not image.is_file():
            pytest.fail(f"firmware build did not produce {image}")


@pytest.fixture(scope="session")
def candidate_binary() -> Path:
    candidate = locate_candidate()
    if candidate is None:
        pytest.fail("could not build or locate the c64xemu candidate binary")
    return candidate


@pytest.fixture(scope="session")
def program_execution(
    request: pytest.FixtureRequest,
    candidate_binary: Path,
    tmp_path_factory: pytest.TempPathFactory,
) -> tuple[list[Case], dict[str, Any], list[Path]]:
    selected_ids = selected_case_ids(request, "test_program_case")
    if selected_case_ids(request, "test_differential_case"):
        selected_ids = None
    return program_cases(
        candidate_binary,
        tmp_path_factory.mktemp("programs"),
        selected_ids,
    )
