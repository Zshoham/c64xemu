#!/usr/bin/env python3
"""Build c64xemu with LLVM PGO, optionally training on one natural emulator run."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def command(argv: list[str], *, env: dict[str, str] | None = None,
            stdout_path: Path | None = None, stderr_path: Path | None = None) -> int:
    print("+ " + " ".join(argv), flush=True)
    if stdout_path is None and stderr_path is None:
        return subprocess.run(argv, cwd=ROOT, env=env).returncode
    with ExitStack() as stack:
        out = stack.enter_context(stdout_path.open("wb")) if stdout_path else None
        err = stack.enter_context(stderr_path.open("wb")) if stderr_path else None
        return subprocess.run(argv, cwd=ROOT, env=env, stdout=out, stderr=err).returncode


def output(argv: list[str]) -> str:
    return subprocess.check_output(argv, cwd=ROOT, text=True).strip()


def source_identity() -> dict[str, str | bool]:
    commit = output(["git", "rev-parse", "HEAD"])
    tree = output(["git", "rev-parse", "HEAD^{tree}"])
    diff = subprocess.check_output(["git", "diff", "HEAD", "--binary"], cwd=ROOT)
    status = output(["git", "status", "--porcelain"])
    tracked = subprocess.check_output(
        ["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=ROOT
    ).split(b"\0")
    working_hash = hashlib.sha256()
    for raw_path in sorted(path for path in tracked if path):
        path = ROOT / os.fsdecode(raw_path)
        if path.is_file():
            working_hash.update(raw_path)
            working_hash.update(b"\0")
            working_hash.update(bytes.fromhex(digest(path)))
    return {
        "commit": commit,
        "tree": tree,
        "dirty": bool(status),
        "working_diff_sha256": hashlib.sha256(diff).hexdigest(),
        "working_tree_sha256": working_hash.hexdigest(),
    }


def validate_release_profile() -> None:
    cargo = (ROOT / "Cargo.toml").read_text()
    release = re.search(r"(?ms)^\[profile\.release\]\s*(.*?)(?=^\[|\Z)", cargo)
    if not release:
        raise RuntimeError("Cargo.toml must define [profile.release]")
    body = release.group(1)
    required = ['lto = "fat"', "codegen-units = 1", 'panic = "abort"']
    missing = [setting for setting in required if setting not in body]
    if missing:
        raise RuntimeError(f"release profile must retain {', '.join(missing)}")


def rust_llvm_tools() -> tuple[Path, str, str]:
    sysroot = Path(output(["rustc", "--print", "sysroot"]))
    rust_info = output(["rustc", "-Vv"])
    host = re.search(r"^host:\s*(\S+)$", rust_info, re.M)
    llvm = re.search(r"^LLVM version:\s*(\S+)$", rust_info, re.M)
    if not host or not llvm:
        raise RuntimeError("could not read rustc host/LLVM version")
    tool = sysroot / "lib/rustlib" / host.group(1) / "bin/llvm-profdata"
    if not tool.is_file():
        raise RuntimeError(f"{tool} is missing; install rustup component llvm-tools-preview")
    version = subprocess.check_output([str(tool), "--version"], text=True).strip()
    tool_release = re.search(r"LLVM version\s+([0-9.]+)", version)
    if not tool_release or tool_release.group(1) != llvm.group(1):
        raise RuntimeError(f"llvm-profdata does not match rustc LLVM {llvm.group(1)}: {version}")
    return tool, llvm.group(1), version


def build_binary(target_dir: Path, target_cpu: str, profile_flag: str) -> tuple[list[str], dict[str, str]]:
    validate_release_profile()
    flags = f"-C target-cpu={target_cpu} {profile_flag}".strip()
    env = {**os.environ, "CARGO_TARGET_DIR": str(target_dir), "RUSTFLAGS": flags}
    # Match the optimized binary build used by upstream conformance CI, while
    # keeping the profile instrumentation/use flags in RUSTFLAGS.
    argv = ["cargo", "build", "--release", "--locked", "--bin", "c64xemu"]
    if command(argv, env=env) != 0:
        raise RuntimeError("cargo build failed")
    return argv, {"CARGO_TARGET_DIR": str(target_dir), "RUSTFLAGS": flags}


def write_receipt(path: Path, receipt: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print(f"receipt: {path}")


def build_from_profile(args: argparse.Namespace) -> None:
    profile = args.profile.resolve()
    elf = args.elf.resolve() if args.elf else None
    if not profile.is_file():
        raise RuntimeError(f"profile does not exist: {profile}")
    tools, llvm_version, profdata_version = rust_llvm_tools()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    target_dir = out / "pgo-target"
    build_argv, build_env = build_binary(
        target_dir, args.target_cpu, f"-C profile-use={profile}"
    )
    binary = target_dir / "release/c64xemu"
    receipt = {
        "schema": "c64xemu-llvm-pgo-build-v1",
        "mode": "profile-use-existing",
        "source": source_identity(),
        "elf": {"path": str(elf), "sha256": digest(elf)} if elf else None,
        "profile": {"path": str(profile), "sha256": digest(profile)},
        "binary": {"path": str(binary), "sha256": digest(binary)},
        "target_cpu": args.target_cpu,
        "release_profile": {"lto": "fat", "codegen_units": 1, "panic": "abort"},
        "rustc_llvm_version": llvm_version,
        "llvm_profdata": {"path": str(tools), "version": profdata_version},
        "build_argv": build_argv,
        "build_environment": build_env,
        "training_argv": None,
    }
    write_receipt(out / "pgo-build-receipt.json", receipt)


def train_and_build(args: argparse.Namespace) -> None:
    elf = args.elf.resolve()
    if not elf.is_file():
        raise RuntimeError(f"ELF does not exist: {elf}")
    if args.cycles <= 0:
        raise RuntimeError("--cycles must be positive")
    if args.cpu is not None and not shutil.which("taskset"):
        raise RuntimeError("--cpu requires taskset")
    profdata_tool, llvm_version, profdata_version = rust_llvm_tools()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    raw_dir = out / "raw"
    raw_dir.mkdir(exist_ok=True)
    profdata = out / "training.profdata"

    instrumented_target = out / "instrumented-target"
    instrumented_build_argv, instrumented_build_env = build_binary(
        instrumented_target, args.target_cpu, f"-C profile-generate={raw_dir}"
    )
    instrumented_binary = instrumented_target / "release/c64xemu"
    state = out / "training-state.json"
    log_out, log_err = out / "training.stdout", out / "training.stderr"
    guest_argv = [
        str(instrumented_binary), "run", str(elf), "--device", args.device,
        "--boot-mode", str(args.boot_mode), "--max-cycles", str(args.cycles),
        "--state-out", str(state),
    ]
    if args.cpu is not None:
        guest_argv = ["taskset", "-c", str(args.cpu), *guest_argv]
    profile_pattern = str(raw_dir / "train-%p-%m.profraw")
    train_env = {**os.environ, "LLVM_PROFILE_FILE": profile_pattern}
    run_code = command(guest_argv, env=train_env, stdout_path=log_out, stderr_path=log_err)
    raw_profiles = sorted(raw_dir.glob("train-*.profraw"))
    if not raw_profiles:
        raise RuntimeError("training produced no run-specific .profraw files")
    stderr_text = log_err.read_text(errors="replace")
    cap_exit = run_code != 0 and re.search(
        rf"cycle limit {args.cycles} reached at PC 0x[0-9a-fA-F]+", stderr_text
    )
    if run_code != 0 and not cap_exit:
        raise RuntimeError(f"training command failed with exit code {run_code}")
    if not state.is_file():
        raise RuntimeError("training did not write its state file")

    merge_argv = [str(profdata_tool), "merge", f"-output={profdata}", *map(str, raw_profiles)]
    if command(merge_argv) != 0:
        raise RuntimeError("llvm-profdata merge failed")
    pgo_target = out / "pgo-target"
    pgo_build_argv, pgo_build_env = build_binary(
        pgo_target, args.target_cpu, f"-C profile-use={profdata}"
    )
    binary = pgo_target / "release/c64xemu"
    receipt = {
        "schema": "c64xemu-llvm-pgo-build-v1",
        "mode": "train-and-build",
        "source": source_identity(),
        "elf": {"path": str(elf), "sha256": digest(elf)},
        "profile": {
            "raw": [{"path": str(p), "sha256": digest(p)} for p in raw_profiles],
            "merged": {"path": str(profdata), "sha256": digest(profdata)},
        },
        "binary": {"path": str(binary), "sha256": digest(binary)},
        "target_cpu": args.target_cpu,
        "release_profile": {"lto": "fat", "codegen_units": 1, "panic": "abort"},
        "rustc_llvm_version": llvm_version,
        "llvm_profdata": {"path": str(profdata_tool), "version": profdata_version},
        "instrumented_build_argv": instrumented_build_argv,
        "instrumented_build_environment": instrumented_build_env,
        "training_argv": guest_argv,
        "training_environment": {"LLVM_PROFILE_FILE": profile_pattern},
        "training_exit": {"code": run_code, "cycle_cap_exit": bool(cap_exit)},
        "training_artifacts": {
            "state_sha256": digest(state),
            "stdout_sha256": digest(log_out),
            "stderr_sha256": digest(log_err),
        },
        "profile_merge_argv": merge_argv,
        "profile_use_build_argv": pgo_build_argv,
        "profile_use_build_environment": pgo_build_env,
    }
    write_receipt(out / "pgo-build-receipt.json", receipt)


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__)
    modes = ap.add_subparsers(dest="mode", required=True)
    train = modes.add_parser("train", help="instrument, run one training prefix, merge, and build PGO")
    train.add_argument("--elf", type=Path, required=True)
    train.add_argument("--device", default="c6455", help="emulated device (default: c6455)")
    train.add_argument("--cycles", type=int, required=True)
    train.add_argument("--boot-mode", type=int, default=0)
    train.add_argument("--cpu", type=int, help="optional taskset CPU affinity")
    train.add_argument("--target-cpu", default="native")
    train.add_argument("--output-dir", type=Path, required=True)
    train.set_defaults(func=train_and_build)
    build = modes.add_parser("build", help="build PGO binary from existing llvm-profdata")
    build.add_argument("--profile", type=Path, required=True)
    build.add_argument("--elf", type=Path, help="optional ELF pin for the build receipt")
    build.add_argument("--target-cpu", default="native")
    build.add_argument("--output-dir", type=Path, required=True)
    build.set_defaults(func=build_from_profile)
    return ap


def main() -> int:
    args = parser().parse_args()
    try:
        args.func(args)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"build_pgo: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
