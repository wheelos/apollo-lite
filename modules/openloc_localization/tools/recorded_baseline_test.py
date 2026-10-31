"""Focused acceptance-report regressions using bounded synthetic evidence."""

import copy
import unittest

from recorded_baseline import compare, scan_frontiers, summarize


class BaselineAcceptanceTest(unittest.TestCase):
    def evidence(self):
        imu = [
            {
                "timestamp_ns": str(1_000_000_000 + i * 5_000_000),
                "receive_ns": str(1_001_000_000 + i * 5_000_000),
            }
            for i in range(2)
        ]
        publications = []
        for i, source in enumerate(imu, 1):
            publications.append(
                {
                    "sequence": str(i),
                    "measurement_s": str(int(source["timestamp_ns"]) / 1e9),
                    "receive_s": str(int(source["receive_ns"]) / 1e9),
                    "age_s": "0.002",
                    "publication_interval_ns": "10000000",
                    "usable": "1",
                    "new_measurement": "1",
                    "reason": "UNCERTAINTY_UNQUALIFIED",
                    **{
                        name: "0"
                        for name in (
                            "px", "py", "pz", "qx", "qy", "qz", "vx", "vy", "vz"
                        )
                    },
                    "qw": "1",
                }
            )
        reference = [{"px": "0", "py": "0", "pz": "0"} for _ in imu]
        return publications, imu, reference

    def test_preserves_source_identity_and_counts_new_not_held_output(self):
        pubs, imu, reference = self.evidence()
        held = dict(pubs[-1], new_measurement="0")
        result = summarize(pubs + [held], imu, reference)
        self.assertEqual(result["messages"], 3)
        self.assertEqual(result["new_usable"], 2)
        self.assertEqual(result["maximum_position_difference_from_sync_m"], 0)
        self.assertFalse(result["accuracy_error_claimed"])
        for key, value in (
            ("measurement_s", "2"),
            ("receive_s", "2"),
            ("age_s", "0.1001"),
            ("px", "nan"),
            ("qw", "2"),
            ("sequence", "9"),
        ):
            changed = copy.deepcopy(pubs)
            changed[-1][key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                summarize(changed, imu, reference)
        with self.assertRaises(ValueError):
            summarize(pubs[:1], imu, reference)

    def test_unavailable_pose_and_duplicate_new_identity_fail(self):
        pubs, imu, reference = self.evidence()
        with self.assertRaises(ValueError):
            summarize(pubs + [pubs[-1]], imu, reference)
        pubs[-1]["usable"] = "0"
        with self.assertRaises(ValueError):
            summarize(pubs, imu, reference)
        for key in ("px", "py", "pz", "qx", "qy", "qz", "qw", "vx", "vy", "vz"):
            pubs[-1][key] = ""
        pubs[-1]["new_measurement"] = "0"
        self.assertEqual(summarize(pubs, imu, reference)["new_usable"], 1)

    def test_scan_frontier_requires_exact_original_receive_and_reference(self):
        before = {
            "sequence": "1",
            "raw_end_ns": "1000000000",
            "source_receive_ns": "1001000000",
            "reference_ns": "1005000000",
            "processing_frontier_receive_ns": "1006000000",
        }
        after = {
            "sequence": "1",
            "raw_end_s": "1",
            "source_receive_s": "1.001",
            "reference_s": "1.005",
            "frontier_receive_s": "1.006",
        }
        scan_frontiers([after], [before])
        for key in after:
            changed = dict(after, **{key: "2"})
            with self.subTest(field=key), self.assertRaises(ValueError):
                scan_frontiers([changed], [before])
        with self.assertRaises(ValueError):
            scan_frontiers([], [before])

    def test_performance_claim_requires_three_pairs_and_declared_threshold(self):
        protocol = {
            "captures": [{"id": "sample"}],
            "optimization_minimum_repeats": 3,
            "optimization_minimum_median_p95_lateness_reduction_fraction": 0.2,
        }
        report = {"protocol": protocol, "runs": []}
        for index in range(3):
            for mode, lateness in (("serialized", 100), ("prefetch", 80)):
                report["runs"].append({
                    "capture": "sample", "repeat": index, "mode": mode,
                    "metrics": {
                        "send_lateness_ns": {"p95": lateness}, "new_usable": 10,
                    },
                    "tasks": {"committed:NONE": 2},
                })
        self.assertFalse(compare(dict(report, runs=report["runs"][:2]))[0][
            "meets_declared_sender_goal"
        ])
        self.assertTrue(compare(report)[0]["meets_declared_sender_goal"])
        report["runs"][-1]["metrics"]["send_lateness_ns"]["p95"] = 100
        report["runs"][-3]["metrics"]["send_lateness_ns"]["p95"] = 100
        self.assertFalse(compare(report)[0]["meets_declared_sender_goal"])
        with self.assertRaises(ValueError):
            compare(dict(report, runs=report["runs"][:-1]))


if __name__ == "__main__":
    unittest.main()
