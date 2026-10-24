{ pkgs, ... }:

let
  tiToolsFhs = pkgs.buildFHSEnv {
    pname = "ti-tools-fhs";
    version = "1";
    # pkgsi686Linux.glibc supplies /lib/ld-linux.so.2, which the 32-bit SYS/BIOS installer
    # needs; tools/fetch_deps.sh runs it inside this environment.
    targetPkgs = pkgs: with pkgs; [ zlib pkgsi686Linux.glibc ];
  };
in
{
  packages = [
    pkgs.git
    pkgs.ripgrep
    pkgs.fd
    pkgs.jq
    pkgs.just
    pkgs.bc
    pkgs.perf
    pkgs.gnumake
    # TI's IBL romparse, built by tests/tools/ibl_tools.py, is a bison grammar and a flex lexer.
    pkgs.bison
    pkgs.flex
    pkgs.cargo-nextest
    pkgs.python3Packages.pytest
    tiToolsFhs
  ];

  languages.rust.enable = true;
  languages.c.enable = true;
  languages.python.enable = true;

  enterTest = "just ci";

}
