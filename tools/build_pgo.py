#!/usr/bin/env python3
"""Build c64xemu with LLVM PGO, optionally training on one natural emulator run.

`train` builds an instrumented release binary, runs one guest ELF to its exit or to
a cycle cap, merges that run's raw profile, and builds the profile-use binary. `build`
does only the last step, from an existing merged profile. Each mode writes a JSON
receipt pinning the source, tools, flags, and artifacts, and needs its own output
directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tomllib
from contextlib import ExitStack
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

SCHEMA = "c64xemu-llvm-pgo-build-v2"
TRAIN_RECEIPT = "pgo-build-receipt.json"
BUILD_RECEIPT = "pgo-build-from-profile-receipt.json"
MERGED_PROFILE = "training.profdata"
TRAINING_STATE = "training-state.json"
TRAINING_STDOUT = "training.stdout"
TRAINING_STDERR = "training.stderr"
RAW_DIR = "raw"
# The release settings PGO results are only comparable under.
REQUIRED_RELEASE = {"lto": "fat", "codegen-units": 1, "panic": "abort"}


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def file_record(path: Path) -> dict[str, str]:
    return {"path": str(path), "sha256": digest(path)}


def command(
    argv: list[str],
    *,
    env: dict[str, str] | None = None,
    stdout_path: Path | None = None,
    stderr_path: Path | None = None,
) -> int:
    """Run `argv` from the repository root, optionally capturing its output to files."""
    print("+ " + shlex.join(argv), flush=True)
    with ExitStack() as stack:
        out = stack.enter_context(stdout_path.open("wb")) if stdout_path else None
        err = stack.enter_context(stderr_path.open("wb")) if stderr_path else None
        return subprocess.run(
            argv, cwd=ROOT, env=env, stdout=out, stderr=err, check=False
        ).returncode


def output(argv: list[str]) -> str:
    return subprocess.check_output(argv, cwd=ROOT, text=True).strip()


def source_identity() -> dict[str, str | bool]:
    """The commit plus a hash of every tracked or untracked, non-ignored file."""
    diff = subprocess.check_output(["git", "diff", "HEAD", "--binary"], cwd=ROOT)
    files = subprocess.check_output(
        ["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=ROOT
    ).split(b"\0")
    working_hash = hashlib.sha256()
    for raw_path in sorted(path for path in files if path):
        path = ROOT / os.fsdecode(raw_path)
        if path.is_file():
            working_hash.update(raw_path + b"\0")
            working_hash.update(bytes.fromhex(digest(path)))
    return {
        "commit": output(["git", "rev-parse", "HEAD"]),
        "tree": output(["git", "rev-parse", "HEAD^{tree}"]),
        "dirty": bool(output(["git", "status", "--porcelain"])),
        "working_diff_sha256": hashlib.sha256(diff).hexdigest(),
        "working_tree_sha256": working_hash.hexdigest(),
    }


def release_profile() -> dict:
    """Cargo.toml's explicit [profile.release] table; it must keep REQUIRED_RELEASE."""
    try:
        manifest = tomllib.loads((ROOT / "Cargo.toml").read_text())
    except tomllib.TOMLDecodeError as exc:
        raise RuntimeError(f"Cargo.toml: {exc}") from exc
    release = manifest.get("profile", {}).get("release")
    if release is None:
        raise RuntimeError("Cargo.toml must define [profile.release]")
    missing = [
        f"{key} = {json.dumps(value)}"
        for key, value in REQUIRED_RELEASE.items()
        if release.get(key) != value
    ]
    if missing:
        raise RuntimeError(f"release profile must retain {', '.join(missing)}")
    return release


def llvm_major(version: str) -> str:
    return version.partition(".")[0]


def find_llvm_profdata(override: str | None, host: str, llvm: str) -> dict | None:
    """The first llvm-profdata whose LLVM major version matches rustc's.

    An explicit override is the only candidate and must be compatible; otherwise the
    rustup llvm-tools copy in rustc's sysroot is preferred over one on PATH.
    """
    if override:
        candidates = [override]
    else:
        sysroot = Path(output(["rustc", "--print", "sysroot"]))
        candidates = [
            str(sysroot / "lib/rustlib" / host / "bin/llvm-profdata"),
            "llvm-profdata",
        ]
    for candidate in candidates:
        resolved = shutil.which(candidate)
        if not resolved:
            continue
        version = output([resolved, "--version"])
        match = re.search(r"LLVM version\s+([0-9.]+)", version)
        if match and llvm_major(match.group(1)) == llvm_major(llvm):
            return {
                "path": str(Path(resolved).resolve()),
                "version": version,
                "exact_llvm_match": match.group(1) == llvm,
            }
        if override:
            raise RuntimeError(
                f"llvm-profdata must match rustc LLVM major {llvm}: {version}"
            )
    return None


def toolchain(explicit: str | None, *, required: bool) -> dict:
    """rustc, Cargo and (when found, or `required`) llvm-profdata identities."""
    rust_info = output(["rustc", "-Vv"])
    fields = dict(
        line.split(": ", 1) for line in rust_info.splitlines() if ": " in line
    )
    missing = [key for key in ("host", "LLVM version") if not fields.get(key)]
    if missing:
        raise RuntimeError(f"rustc -Vv missing required fields: {', '.join(missing)}")
    host, llvm = fields["host"], fields["LLVM version"]
    override = explicit or os.environ.get("LLVM_PROFDATA")
    profdata = find_llvm_profdata(override, host, llvm)
    if profdata is None and (required or override):
        raise RuntimeError(
            "no compatible llvm-profdata found; use --llvm-profdata or LLVM_PROFDATA"
        )
    return {
        "rustc_verbose": rust_info,
        "rustc": fields,
        "cargo_version": output(["cargo", "-V"]),
        "host": host,
        "rustc_llvm_version": llvm,
        "llvm_profdata": profdata,
    }


def resolve_target_cpu(target_cpu: str) -> str:
    if target_cpu != "native":
        return target_cpu
    cpus = output(["rustc", "--print", "target-cpus"])
    native = re.search(r"native\s+-.*?currently\s+([^)]*)", cpus)
    if not native:
        raise RuntimeError("could not resolve native target CPU")
    return native.group(1)


def check_output_dir(out: Path, *, training: bool) -> None:
    """Refuse a directory that would leak into the source hash or the other mode."""
    if out.is_relative_to(ROOT):
        probe = out.relative_to(ROOT) / "__pgo_probe__"
        ignored = subprocess.run(
            ["git", "check-ignore", "-q", str(probe)], cwd=ROOT, check=False
        )
        if ignored.returncode:
            raise RuntimeError("output directory inside repository must be gitignored")
    if training and (out / BUILD_RECEIPT).exists():
        raise RuntimeError(
            "training mode requires a separate output directory from build mode"
        )
    if not training and any(
        (out / name).exists() for name in (TRAIN_RECEIPT, MERGED_PROFILE)
    ):
        raise RuntimeError(
            "build mode requires a separate output directory from training"
        )


def clean_outputs(out: Path, *, training: bool) -> None:
    """Remove this mode's previous results, so none can pass for this run's."""
    out.mkdir(parents=True, exist_ok=True)
    if not training:
        (out / BUILD_RECEIPT).unlink(missing_ok=True)
        return
    for name in (
        TRAIN_RECEIPT,
        TRAINING_STATE,
        TRAINING_STDOUT,
        TRAINING_STDERR,
        MERGED_PROFILE,
    ):
        (out / name).unlink(missing_ok=True)
    if (out / RAW_DIR).exists():
        shutil.rmtree(out / RAW_DIR)
    (out / RAW_DIR).mkdir()


def prepare(args: argparse.Namespace, *, training: bool) -> tuple[Path, dict]:
    """Validate everything, then clear stale outputs; returns the receipt's base."""
    out = args.output_dir.resolve()
    check_output_dir(out, training=training)
    overrides = sorted(
        key for key in os.environ if key.startswith("CARGO_PROFILE_RELEASE_")
    )
    if overrides:
        raise RuntimeError(
            f"release profile environment overrides are unsupported: {overrides}"
        )
    manifest = release_profile()
    info = toolchain(args.llvm_profdata, required=training)
    info.update(
        source=source_identity(),
        target_cpu=args.target_cpu,
        resolved_target_cpu=resolve_target_cpu(args.target_cpu),
        release_profile_manifest=manifest,
    )
    clean_outputs(out, training=training)
    return out, info


def build_binary(
    target_dir: Path, target_cpu: str, profile_flag: str, host: str
) -> tuple[Path, list[str], dict]:
    """Build the release binary; returns its path, Cargo argv and flag environment.

    Flags go through CARGO_ENCODED_RUSTFLAGS, which Cargo prefers over RUSTFLAGS and
    which keeps paths with spaces intact. `--target` keeps the flags off build scripts
    and proc-macros, as the rustc book's PGO chapter recommends.
    """
    flags = ["-C", f"target-cpu={target_cpu}", "-C", profile_flag]
    encoded = "\x1f".join(flags)
    env = {
        **os.environ,
        "CARGO_TARGET_DIR": str(target_dir),
        "CARGO_ENCODED_RUSTFLAGS": encoded,
    }
    env.pop("RUSTFLAGS", None)
    argv = [
        "cargo",
        "build",
        "--release",
        "--locked",
        "--bin",
        "c64xemu",
        "--target",
        host,
    ]
    if command(argv, env=env) != 0:
        raise RuntimeError("cargo build failed")
    environment = {
        "CARGO_TARGET_DIR": str(target_dir),
        "CARGO_ENCODED_RUSTFLAGS": encoded,
        "rustflags": flags,
        "RUSTFLAGS": None,
    }
    return target_dir / host / "release/c64xemu", argv, environment


def write_receipt(path: Path, receipt: dict) -> None:
    """Write `receipt` atomically, provided the source did not change while building."""
    if source_identity() != receipt["source"]:
        raise RuntimeError("source identity changed during build")
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)
    print(f"binary: {receipt['binary']['path']}")
    print(f"receipt: {path}")


def build_from_profile(args: argparse.Namespace) -> None:
    profile = args.profile.resolve()
    if not profile.is_file():
        raise RuntimeError(f"profile does not exist: {profile}")
    elf = args.elf.resolve() if args.elf else None
    out, info = prepare(args, training=False)
    binary, build_argv, build_env = build_binary(
        out / "pgo-target", args.target_cpu, f"profile-use={profile}", info["host"]
    )
    receipt = {
        "schema": SCHEMA,
        "mode": "profile-use-existing",
        **info,
        "elf_note": {**file_record(elf), "informational": True} if elf else None,
        "profile": file_record(profile),
        "binary": file_record(binary),
        "build_argv": build_argv,
        "build_environment": build_env,
        "training_argv": None,
    }
    write_receipt(out / BUILD_RECEIPT, receipt)


def is_cycle_cap_exit(code: int, stderr_text: str, cycles: int) -> bool:
    """Whether the run hit the cap: exit 1, with the emulator's report as last line."""
    lines = [line for line in stderr_text.splitlines() if line.strip()]
    report = rf"c64xemu: cycle limit {cycles} reached at PC 0x[0-9a-fA-F]+"
    return code == 1 and bool(lines) and re.fullmatch(report, lines[-1]) is not None


def train_and_build(args: argparse.Namespace) -> None:
    elf = args.elf.resolve()
    if not elf.is_file():
        raise RuntimeError(f"ELF does not exist: {elf}")
    if args.cycles <= 0:
        raise RuntimeError("--cycles must be positive")
    if args.cpu is not None and not shutil.which("taskset"):
        raise RuntimeError("--cpu requires taskset")
    out, info = prepare(args, training=True)
    raw_dir = out / RAW_DIR
    profdata = out / MERGED_PROFILE
    state = out / TRAINING_STATE
    log_out, log_err = out / TRAINING_STDOUT, out / TRAINING_STDERR

    instrumented, instrumented_argv, instrumented_env = build_binary(
        out / "instrumented-target",
        args.target_cpu,
        f"profile-generate={raw_dir}",
        info["host"],
    )
    # Nothing else may write here, or this run's profile could not be told apart.
    if any(raw_dir.iterdir()):
        raise RuntimeError("raw directory is not empty before training")
    guest_argv = [
        str(instrumented), "run", str(elf),
        "--device", args.device,
        "--boot-mode", str(args.boot_mode),
        "--max-cycles", str(args.cycles),
        "--state-out", str(state),
    ]  # fmt: skip
    if args.cpu is not None:
        guest_argv = ["taskset", "-c", str(args.cpu), *guest_argv]
    profile_pattern = str(raw_dir / "train-%p-%m.profraw")
    run_code = command(
        guest_argv,
        env={**os.environ, "LLVM_PROFILE_FILE": profile_pattern},
        stdout_path=log_out,
        stderr_path=log_err,
    )

    raw_profiles = list(raw_dir.iterdir())
    if (
        len(raw_profiles) != 1
        or not raw_profiles[0].match("train-*.profraw")
        or not raw_profiles[0].stat().st_size
    ):
        raise RuntimeError(
            "training must produce exactly one non-empty run-specific .profraw file"
        )
    stderr_text = log_err.read_text(errors="replace")
    cap_exit = is_cycle_cap_exit(run_code, stderr_text, args.cycles)
    if "terminated by SIGTERM" in stderr_text or (run_code != 0 and not cap_exit):
        raise RuntimeError(f"training command failed with exit code {run_code}")
    if not state.is_file():
        raise RuntimeError("training did not write its state file")

    merge_argv = [
        info["llvm_profdata"]["path"],
        "merge",
        f"-output={profdata}",
        str(raw_profiles[0]),
    ]
    if command(merge_argv) != 0:
        raise RuntimeError("llvm-profdata merge failed")
    binary, pgo_argv, pgo_env = build_binary(
        out / "pgo-target", args.target_cpu, f"profile-use={profdata}", info["host"]
    )
    receipt = {
        "schema": SCHEMA,
        "mode": "train-and-build",
        **info,
        "elf": file_record(elf),
        "profile": {
            "raw": [file_record(raw_profiles[0])],
            "merged": file_record(profdata),
        },
        "binary": file_record(binary),
        "instrumented_build_argv": instrumented_argv,
        "instrumented_build_environment": instrumented_env,
        "training_argv": guest_argv,
        "training_environment": {"LLVM_PROFILE_FILE": profile_pattern},
        "training_exit": {"code": run_code, "cycle_cap_exit": cap_exit},
        "training_artifacts": {
            "state_sha256": digest(state),
            "stdout_sha256": digest(log_out),
            "stderr_sha256": digest(log_err),
        },
        "profile_merge_argv": merge_argv,
        "profile_use_build_argv": pgo_argv,
        "profile_use_build_environment": pgo_env,
    }
    write_receipt(out / TRAIN_RECEIPT, receipt)


def add_build_options(mode: argparse.ArgumentParser, profdata_help: str) -> None:
    mode.add_argument("--target-cpu", default="native")
    mode.add_argument("--output-dir", type=Path, required=True)
    mode.add_argument("--llvm-profdata", help=f"{profdata_help} (also LLVM_PROFDATA)")


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    modes = ap.add_subparsers(dest="mode", required=True)

    train = modes.add_parser(
        "train", help="instrument, run one training prefix, merge, and build PGO"
    )
    train.add_argument("--elf", type=Path, required=True)
    train.add_argument(
        "--device", default="c6455", help="emulated device (default: c6455)"
    )
    train.add_argument("--cycles", type=int, required=True)
    train.add_argument("--boot-mode", type=int, default=0)
    train.add_argument("--cpu", type=int, help="optional taskset CPU affinity")
    add_build_options(train, "llvm-profdata path")
    train.set_defaults(func=train_and_build)

    build = modes.add_parser(
        "build", help="build PGO binary from existing llvm-profdata"
    )
    build.add_argument("--profile", type=Path, required=True)
    build.add_argument(
        "--elf", type=Path, help="ELF recorded in the receipt as a note only"
    )
    add_build_options(build, "optional llvm-profdata path")
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
