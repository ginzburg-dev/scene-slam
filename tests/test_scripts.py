import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ScriptTests(unittest.TestCase):
    def test_evaluate_prints_metric_scale_commands(self):
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts" / "evaluate.py"),
                "--groundtruth",
                "groundtruth.txt",
                "--trajectory",
                "trajectory_tum.txt",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("evo_ape tum groundtruth.txt trajectory_tum.txt --align", result.stdout)
        self.assertIn("evo_rpe tum groundtruth.txt trajectory_tum.txt --align", result.stdout)
        self.assertNotIn("--correct_scale", result.stdout)

    def test_renderer_reports_valid_trajectory_extent_without_plot_dependencies(self):
        with tempfile.TemporaryDirectory() as directory:
            trajectory = Path(directory) / "trajectory.txt"
            trajectory.write_text(
                "1.0 0 0 0 0 0 0 1\n2.0 2 1 -3 0 0 0 1\n",
                encoding="utf-8",
            )
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "render_trajectory.py"),
                    str(trajectory),
                    "--summary-only",
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertIn("poses=2", result.stdout)
            self.assertIn("x_range_m=2", result.stdout)
            self.assertIn("z_range_m=3", result.stdout)


if __name__ == "__main__":
    unittest.main()
