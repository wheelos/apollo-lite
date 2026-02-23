# Apollo Configuration Override Design

## Overview

Apollo's configuration system uses a **layered override model** to separate
default (in-source) values from environment-specific or persistent overrides,
without ever mutating the checked-in defaults.

---

## Motivation

The previous approach had two problems:

1. **No parameter-service integration.** Runtime parameters stored in the Cyber
   `ParameterServer` could not influence module configs.
2. **Symbolic-link pollution.** Operators had to symlink files into the source
   tree to override defaults, risking accidental commits.

---

## Override Priority (Lowest → Highest)

| Tier | Source | Location |
|------|--------|----------|
| 1 | **Defaults** | In-source path, e.g. `/apollo/modules/control/conf/control_conf.pb.txt` |
| 2 | **External persistent overrides** | `/data/conf/<same filename>` |

> **Future tier 3 – Parameter Server** (planned):  
> Modules that register as `ParameterClient`s will receive the highest-priority
> values pushed by a `ParameterServer` at runtime.  
> Implementation tracked separately once the file-based tiers are stable.

---

## Merge Semantics

Overrides are applied using
[`google::protobuf::Message::MergeFrom()`](https://protobuf.dev/reference/cpp/api-docs/google.protobuf.message/#Message.MergeFrom.details):

* **Scalar fields** (int, float, bool, enum, string): set in the override file
  wins; unset fields in the override retain the default value.
* **Repeated fields**: entries from the override are *appended* to the default
  list.
* **Nested messages**: merged recursively (deep merge).

This is intentionally a *non-destructive* merge: only the fields explicitly
present in the override file change.

---

## Implementation

The entry point is `cyber::common::GetProtoFromFileWithOverride()` defined in
[`cyber/common/config_loader.h`](../cyber/common/config_loader.h):

```cpp
#include "cyber/common/config_loader.h"

MyConfig cfg;
ACHECK(cyber::common::GetProtoFromFileWithOverride(
    FLAGS_my_conf_file, &cfg))
    << "Failed to load config";
```

Internally it:

1. Calls `GetProtoFromFile(default_path, &msg)` to load the default.
2. Derives the override filename as the **basename** of `default_path`.
3. If `/data/conf/<basename>` exists, parses it into a temporary message and
   calls `msg.MergeFrom(override)`.

---

## Writing an Override File

Create a partial `.pb.txt` file at `/data/conf/<original-filename>` containing
**only the fields you want to change**:

```protobuf
# /data/conf/control_conf.pb.txt
max_steering_angle: 480.0   # override default of 470.0
```

Fields omitted from this file retain their in-source defaults.

---

## Security Guidelines

* `/data/conf/` should be writable only by the system operator (root or a
  dedicated `apollo` service account).  File permissions: `0640`.
* **Never** store secrets (API tokens, encryption keys) in source-tree defaults.
  Put secrets exclusively in `/data/conf/` with appropriate filesystem
  permissions.
* The default config files committed to the repository must not contain
  credentials, license keys, or private network addresses.

---

## Module Adoption

Replace direct `GetProtoFromFile` calls in module `Init()` functions:

```cpp
// Before
ACHECK(cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_));

// After
ACHECK(cyber::common::GetProtoFromFileWithOverride(
    FLAGS_control_conf_file, &control_conf_));
```

The `modules/control/control_component.cc` module has already been updated as
the reference implementation.

---

## Directory Layout

```
/apollo/                      ← source tree (read-only in production)
  modules/
    control/conf/
      control_conf.pb.txt     ← tier-1 defaults

/data/conf/                   ← persistent overrides (operator-managed)
  control_conf.pb.txt         ← tier-2: partial override (optional)
```
