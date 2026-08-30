# Index (current tools)

- `check_cameras.sh` — inspect and report attached camera hardware and device nodes.
- `gnss_conf.py` — configure GNSS settings and manage GNSS-related configuration.
- `straight_eval.py` — run straight-line evaluation tests (drive/record analysis utilities).
- `save_lidar_semantic_pcd.py` — pair lidar input and save labeled points as PCD.

**How to get help**

- Run the tool with `-h`/`--help`, for example:

```sh
./check_cameras.sh --help
python gnss_conf.py --help
```

If a tool lacks a help flag, inspect its header: `head -n 20 <tool>`.

---

**Principle (short)**

- Keep tools single-file and self-describing; README is an index and guideline only.
- Provide built-in help, clear shebangs, minimal deps, safe defaults, and meaningful exit codes.
- Add a one-line index entry when adding new tools.
