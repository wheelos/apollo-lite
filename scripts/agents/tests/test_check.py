import tempfile
import unittest
from pathlib import Path

from scripts.agents.check import check, check_links, skill_metadata


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def skill(self, metadata):
        path = self.root / "sample/SKILL.md"
        path.parent.mkdir(exist_ok=True)
        path.write_text(f"---\n{metadata}\n---\n# Sample\n", encoding="utf-8")
        return path

    def test_existing_scalar_formats_and_directory_identity(self):
        for metadata in (
            "name: sample\ndescription: Use for a sample task.",
            'name: "sample"\ndescription: "Task: check the sample."',
            "name: 'sample'\ndescription: 'Check the sample.'",
        ):
            with self.subTest(metadata=metadata):
                self.assertEqual("sample", skill_metadata(self.skill(metadata))["name"])
        with self.assertRaisesRegex(ValueError, "mismatch"):
            skill_metadata(self.skill("name: other\ndescription: Check sample."))

    def test_bad_frontmatter_is_not_discovered_as_valid(self):
        for metadata in (
            "name: sample",
            "name: sample\nname: sample\ndescription: Task",
            "name: sample\ndescription: ''",
            "name: sample\ndescription: >",
            "name: sample\ndescription: Task: invalid scalar",
            "name: sample\ndescription: null",
            "name: sample\ndescription: 'broken' quote'",
        ):
            with self.subTest(metadata=metadata), self.assertRaises(ValueError):
                skill_metadata(self.skill(metadata))

    def test_links_defer_external_but_fail_missing_local(self):
        path = self.root / "AGENTS.md"
        path.write_text(
            "[local](missing.md)\n[context](ad_agents/context/README.md)\n"
            "`ad_agents/context/contract.md`\n[web](https://example.com)\n",
            encoding="utf-8",
        )
        errors, deferred = check_links(path, self.root)
        self.assertEqual(1, len(errors))
        self.assertEqual({"README.md", "contract.md"}, deferred)
        context = self.root / "ad_agents/context"
        context.mkdir(parents=True)
        (context / "README.md").write_text("# Context\n", encoding="utf-8")
        errors, deferred = check_links(path, self.root, context)
        self.assertEqual(2, len(errors))
        self.assertEqual(set(), deferred)

    def test_relative_context_links_are_also_checked(self):
        path = self.root / ".agents/skills/sample/SKILL.md"
        path.parent.mkdir(parents=True)
        path.write_text(
            "[contract](../../../ad_agents/context/contract.md)", encoding="utf-8"
        )
        errors, deferred = check_links(path, self.root)
        self.assertFalse(errors)
        self.assertEqual({"contract.md"}, deferred)

    def test_missing_skill_index_and_manifest_fail_configuration(self):
        skill = self.root / ".agents/skills/sample/SKILL.md"
        skill.parent.mkdir(parents=True)
        skill.write_text(
            "---\nname: sample\ndescription: Check sample.\n---\n", encoding="utf-8"
        )
        report = check(self.root)
        self.assertEqual("failed", report["status"])
        self.assertIn(
            "missing external checkout manifest: ad_agents.repos", report["errors"]
        )
        (self.root / "AGENTS.md").write_text("# No index\n", encoding="utf-8")
        report = check(self.root)
        self.assertIn("AGENTS.md: skill not indexed: sample", report["errors"])
