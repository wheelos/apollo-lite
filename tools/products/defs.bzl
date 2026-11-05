"""Compatibility export; Core owns the product assembly contract."""

load("@core//runtime:product.bzl", _runtime_product = "runtime_product")

runtime_product = _runtime_product
