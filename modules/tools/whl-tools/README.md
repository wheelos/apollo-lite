# Index (current tools)

- `check_cameras.sh` — inspect and report attached camera hardware and device nodes.
- `gnss_conf.py` — configure GNSS settings and manage GNSS-related configuration.
- `image_message_publisher.cc` — publish decoded image-directory or video frames as `rgb8` Cyber image messages.
- `lane_debug_visualizer.py` — strictly pair camera images and lane messages by timestamp, then save annotated PPM overlays.
- `straight_eval.py` — run straight-line evaluation tests (drive/record analysis utilities).

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
