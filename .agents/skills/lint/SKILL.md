---
name: "lint"
description: "Apollo-Lite lint and formatting workflow. Use when checking Python, shell, C++, or Bazel formatting and lint in the managed dev container."
---

# Lint and Formatting

## When

Use this when checking source formatting or lint before submitting changes.

## Prerequisites

- From the repository root, enter the managed development container:

  ```bash
  bash docker/scripts/whl.sh enter
  ```

- `enter` defaults to `dev` and starts it if needed. Run lint from `/apollo`
  as the mapped non-root user.

## Steps

- Run all repository lint and formatting checks with:

  ```bash
  ./apollo.sh lint --all
  ```

  `./apollo.sh lint` without a check selector is invalid; the underlying script
  requires `--all` or one or more specific checks.
- To narrow checks, pass supported selectors such as `--py`, `--sh`,
  `--cpp-format`, `--cpp-lint`, or `--bazel`. Use `--diff <base>` for CI-style
  checks against committed changes since a known base; this mode compares the
  base to `HEAD`, not uncommitted working-tree changes. Use `--all` to include
  local edits.
- The full check covers Python formatting/lint (Black, isort, Flake8), shell
  lint (ShellCheck), C++ formatting (Clang-Format), Bazel cpplint, and Bazel
  formatting (Buildifier).
- The wrapper invokes check-only modes. Review reported changes and make
  formatting edits only when they are within the requested scope.

## Acceptance

- Selected checks exit successfully and cover the changed file types.
- For uncommitted edits, use working-tree checks rather than relying on
  `--diff <base>`.
- A skipped check or missing tool is not a successful lint result.

## Failure handling

- Stop and report the first actionable error. Do not install packages, alter
  lint configuration, or retry with changed options without a validation reason
  and permission where scope expands.
- Do not run Apollo lint or Bazel outside the managed container.
- Do not run Bazel as root or bypass the shared repository and disk caches.
- Do not use `apollo.sh clean` to address lint failures.

## Sources (SSOT)

- `apollo.sh`
- `scripts/ci/apollo_lint.sh`
- `docker/scripts/whl.sh`
