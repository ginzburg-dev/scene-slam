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

    def test_viewer_validates_scene_without_opening_ui(self):
        with tempfile.TemporaryDirectory() as directory:
            result_dir = Path(directory)
            (result_dir / "map.ply").write_text(
                "ply\nformat ascii 1.0\nelement vertex 2\n"
                "property float x\nproperty float y\nproperty float z\n"
                "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                "end_header\n0 0 1 255 0 0\n1 0 1 0 255 0\n",
                encoding="utf-8",
            )
            (result_dir / "trajectory_tum.txt").write_text(
                "1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0 1\n",
                encoding="utf-8",
            )
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "view_map.py"),
                    str(result_dir),
                    "--summary-only",
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertIn("mode=result points=2 poses=2", result.stdout)

            recording = result_dir / "result.rrd"
            saved = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "view_map.py"),
                    str(result_dir),
                    "--save",
                    str(recording),
                    "--frustum-step",
                    "1",
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertEqual(saved.stderr, "")
            self.assertGreater(recording.stat().st_size, 0)

    def test_viewer_demo_is_available_without_dataset(self):
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts" / "view_map.py"),
                "--demo",
                "--summary-only",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertRegex(result.stdout, r"mode=synthetic points=[1-9][0-9]+ poses=[1-9][0-9]+")

    def test_viewer_can_save_a_rerun_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            recording = Path(directory) / "demo.rrd"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "view_map.py"),
                    "--demo",
                    "--save",
                    str(recording),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertGreater(recording.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
