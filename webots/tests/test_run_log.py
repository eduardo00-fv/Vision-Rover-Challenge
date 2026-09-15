"""El artefacto de corrida debe poder consumirse como NDJSON estándar."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

WEBOTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEBOTS / "controllers" / "vision_supervisor"))

from run_log import RunLog  # noqa: E402


class RunLogTest(unittest.TestCase):
    def test_writes_metadata_events_and_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            log = RunLog(directory, {"id": "unit"}, {"grid": {"cols": 1}})
            log.write("phase", phase="RUNNING")
            log.close(status="INCONCLUSIVE", reason="unit test")
            lines = [json.loads(line) for line in log.path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual([line["type"] for line in lines], ["metadata", "phase", "summary"])
        self.assertEqual(lines[-1]["status"], "INCONCLUSIVE")


if __name__ == "__main__":
    unittest.main()
