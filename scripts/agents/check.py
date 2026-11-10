#!/usr/bin/env python3
"""Check repository Markdown links, agent entrypoints, and skill metadata."""

import argparse
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[2]
LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)\s]+)\)")
CONTEXT = "ad_agents/context/"
PKGS = "pkgs/"
EXCLUDED_DOCUMENT_DIRS = {".git", ".cache", "ad_agents", "pkgs"}


def skill_metadata(path):
    """Validate the portable scalar subset used by this repository's skills."""
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0] != "---" or "---" not in lines[1:]:
        raise ValueError(f"{path}: missing YAML frontmatter")
    values = {}
    for line in lines[1 : lines.index("---", 1)]:
        if not line.strip():
            continue
        match = re.fullmatch(r"(name|description):\s+(.+)", line)
        if not match:
            raise ValueError(f"{path}: unsupported metadata; expected scalar fields")
        key, value = match.groups()
        if key in values:
            raise ValueError(f"{path}: duplicate {key}")
        if value.startswith('"'):
            value = json.loads(value)
        elif value.startswith("'"):
            if not re.fullmatch(r"'(?:[^']|'')*'", value):
                raise ValueError(f"{path}: invalid quoted scalar")
            value = value[1:-1].replace("''", "'")
        elif ": " in value or value.startswith((">", "|", "[", "{", "#", "&", "*")):
            raise ValueError(f"{path}: quote the scalar value")
        elif value.lower() in ("true", "false", "null", "~") or re.fullmatch(
            r"[-+]?\d+(?:\.\d+)?", value
        ):
            raise ValueError(f"{path}: metadata must be strings; quote the value")
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{path}: empty {key}")
        values[key] = value
    name = values.get("name", "")
    if (
        not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", name)
        or len(name) > 64
        or name != path.parent.name
    ):
        raise ValueError(f"{path}: invalid name or directory mismatch")
    if not 1 <= len(values.get("description", "")) <= 1024:
        raise ValueError(f"{path}: invalid description")
    return values


def markdown_anchors(path):
    """Return permissive anchor keys for Markdown headings in a file."""
    anchors = set()
    counts = {}
    text = path.read_text(encoding="utf-8")
    for match in re.finditer(r"^#{1,6}\s+(.*?)\s*#*\s*$", text, re.MULTILINE):
        heading = match.group(1)
        heading = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", heading)
        heading = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", heading)
        heading = re.sub(r"<[^>]+>", "", heading)
        key = re.sub(r"[^0-9A-Za-z\u0080-\uffff]+", "", heading).casefold()
        if not key:
            continue
        count = counts.get(key, 0)
        anchors.add(key)
        if count:
            anchors.add(f"{key}{count}")
        counts[key] = count + 1
    return anchors


def check_links(path, root, context_root=None):
    errors = []
    external = set()
    root = Path(root).resolve()
    path = Path(path).resolve()
    text = path.read_text(encoding="utf-8")
    targets = LINK.findall(text)
    targets += re.findall(r"`(ad_agents/context/[^`\s]+)`", text)
    for target in targets:
        if urlsplit(target).scheme:
            continue
        target, _, fragment = target.partition("#")
        target = unquote(target)
        if not target and not fragment:
            continue
        if target and any(char in target for char in "<>*"):
            continue
        candidate = (
            root / target
            if target.startswith(CONTEXT)
            else path.parent / target
            if target
            else path
        ).resolve()
        mount = (root / CONTEXT).resolve()
        if candidate.is_relative_to(mount):
            relative = candidate.relative_to(mount)
            if context_root is None:
                external.add(str(relative))
                continue
            candidate = context_root / relative
        packages = (root / PKGS).resolve()
        if candidate.is_relative_to(packages) and not candidate.exists():
            external.add(str(candidate.relative_to(root)))
            continue
        if not candidate.exists():
            label = path.relative_to(root) if path.is_relative_to(root) else path
            errors.append(f"{label}: missing link {target}")
            continue
        if fragment and candidate.suffix.lower() == ".md":
            anchor = re.sub(
                r"[^0-9A-Za-z\u0080-\uffff]+", "", unquote(fragment)
            ).casefold()
            if anchor not in markdown_anchors(candidate):
                label = path.relative_to(root) if path.is_relative_to(root) else path
                if target:
                    display_target = target
                elif path.is_relative_to(root):
                    display_target = path.relative_to(root).as_posix()
                else:
                    display_target = str(path)
                errors.append(
                    f"{label}: missing anchor #{fragment} in {display_target}"
                )
    return errors, external


def document_paths(root):
    """Return repository Markdown pages while excluding external checkouts."""
    paths = []
    for path in root.rglob("*.md"):
        relative = path.relative_to(root)
        if any(
            part in EXCLUDED_DOCUMENT_DIRS or part.startswith("bazel-")
            for part in relative.parts
        ):
            continue
        paths.append(path)
    return paths


def check(root, context_root=None):
    errors = []
    external = set()
    required_paths = [
        root / "AGENTS.md",
        root / ".github/copilot-instructions.md",
        root / "scripts/agents/README.md",
    ]
    paths = sorted(set(required_paths + document_paths(root)))
    for path in paths:
        if not path.is_file():
            errors.append(f"missing entrypoint: {path}")
            continue
        if path.name == "SKILL.md":
            try:
                skill_metadata(path)
            except ValueError as error:
                errors.append(str(error))
        found, deferred = check_links(path, root, context_root)
        errors.extend(found)
        external.update(deferred)
    skills = sorted((root / ".agents/skills").glob("*/SKILL.md"))
    if not skills:
        errors.append("no skills discovered")
    for entrypoint in ("AGENTS.md", ".agents/skills/README.md"):
        path = root / entrypoint
        if path.is_file():
            text = path.read_text(encoding="utf-8")
            for skill in skills:
                if f"{skill.parent.name}/SKILL.md" not in text:
                    errors.append(
                        f"{entrypoint}: skill not indexed: {skill.parent.name}"
                    )
    if not (root / "wheelos.repos").is_file():
        errors.append("missing external checkout manifest: wheelos.repos")
    if context_root is not None:
        catalog = context_root / "context-catalog.json"
        if not catalog.is_file():
            errors.append(f"missing context catalog: {catalog}")
        else:
            data = json.loads(catalog.read_text(encoding="utf-8"))
            ids = [entry["id"] for entry in data["entries"]]
            if len(ids) != len(set(ids)):
                errors.append("duplicate context catalog IDs")
            targets = data["canonical_entrypoints"] + [
                entry["path"] for entry in data["entries"]
            ]
            for target in targets:
                resolved = (context_root / target).resolve()
                if not resolved.is_relative_to(context_root) or not resolved.is_file():
                    errors.append(f"invalid context catalog path: {target}")
            docs = {
                context_root / "README.md",
                context_root / "framework/README.md",
                context_root / "framework/retrieval/ai-coding-session-entrypoints.md",
                context_root / "framework/build/build-and-test-command-registry.md",
                context_root / "framework/validation/README.md",
            }
            docs.update(context_root / target for target in targets)
            for path in sorted(docs):
                if path.suffix == ".md" and path.is_file():
                    found, _ = check_links(path, root, context_root)
                    errors.extend(found)
    return {
        "status": "failed" if errors else "passed",
        "skills": len(skills),
        "external_context": "checked" if context_root else "not_checked",
        "deferred_context_links": sorted(external),
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--context-root", type=Path)
    args = parser.parse_args()
    try:
        report = check(
            args.root.resolve(),
            args.context_root.resolve() if args.context_root else None,
        )
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({"status": "invalid", "error": str(error)}))
        return 2
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
