import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import pydrivebench


ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = ROOT / "scenarios"


def command(script, *args, status=0):
    result = subprocess.run([sys.executable, ROOT / "tools" / script, *args],
                            capture_output=True, text=True, check=False)
    if result.returncode != status:
        raise AssertionError(f"{script}: expected exit {status}, got {result.returncode}\n{result.stderr}")
    return result


class PipelineTest(unittest.TestCase):
    def test_api(self):
        path = SCENARIOS / "pedestrian_crossing.json"
        first = pydrivebench.run_scenario(path, "driver_v1", 42)
        second = pydrivebench.run_scenario(str(path), "driver_v1", 42)
        self.assertTrue(first.verdict.passed)
        self.assertEqual(first.trajectory_bytes(), second.trajectory_bytes())
        self.assertIsNone(first.metrics["min_distance"]["vehicle"])
        self.assertNotEqual(first.trajectory[0]["actors"], first.trajectory[0]["perceived"])
        with self.assertRaisesRegex(ValueError, "route"):
            pydrivebench.run_scenario(ROOT / "core/tests/fixtures/missing_route.json", "driver_v1", 42)

    def test_cli_pipeline(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            first, second = directory / "first", directory / "second"
            base = SCENARIOS / "pedestrian_crossing.json"
            for output in (first, second):
                command("vary_scenario.py", base, output, "--count", "100", "--seed", "42")
            variants = sorted(first.glob("*.json"))
            self.assertEqual(len(variants), 100)
            self.assertEqual(len({json.loads(path.read_bytes())["agents"][0]["cruise_speed"] for path in variants}), 100)
            for path in variants:
                self.assertEqual(path.read_bytes(), (second / path.name).read_bytes())
            result = command("run_suite.py", first)
            self.assertEqual(json.loads(result.stdout)["summary"]["runs"], 100)
            comparison = directory / "comparison"
            command("compare_versions.py", SCENARIOS, comparison, "--fail-on-regression", status=1)
            report = json.loads((comparison / "comparison.json").read_bytes())
            self.assertEqual(report["baseline"]["runs"], 20)
            self.assertEqual(report["failure_delta"]["collision"], 2)
            pairs = {pair["input"]: pair for pair in report["pairs"]}
            self.assertIn("collision", pairs["cyclist_lane_entry.json"]["resolved_failures"])
            self.assertIn("collision", pairs["pedestrian_crossing.json"]["new_failures"])
            for pair in pairs.values():
                if "replay" in pair:
                    replay = json.loads((comparison / pair["replay"]).read_bytes())
                    self.assertTrue(replay["baseline"] and replay["candidate"])


if __name__ == "__main__":
    unittest.main()