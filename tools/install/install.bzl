"""Compatibility exports for the extracted legacy installation rules."""

load(
    "@hardware.build//tools/install:install.bzl",
    _install = "install",
    _install_files = "install_files",
    _install_src_files = "install_src_files",
)

install = _install
install_files = _install_files
install_src_files = _install_src_files
