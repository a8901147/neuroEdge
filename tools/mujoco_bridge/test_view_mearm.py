"""view_mearm.py: the free-standing MeArm model viewer -- loads the scene and
steps it until the window is closed. Fake viewer, no window."""
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_run_mearm_preview as prev  # noqa: E402  (FakeViewer)
import view_mearm  # noqa: E402


class ViewMearmTest(unittest.TestCase):
    def test_the_scene_file_it_opens_exists(self):
        self.assertTrue(view_mearm.SCENE_XML.exists(), view_mearm.SCENE_XML)

    def test_main_loads_the_four_actuator_model_and_steps_it_until_the_window_closes(self):
        captured = {}

        def fake_launch(model, data):
            captured["model"], captured["data"] = model, data
            return prev.FakeViewer(50)

        with mock.patch.object(view_mearm.mujoco.viewer, "launch_passive", fake_launch):
            view_mearm.main()
        self.assertEqual([captured["model"].actuator(i).name for i in range(captured["model"].nu)],
                         ["base", "shoulder", "elbow", "claw"])
        self.assertGreater(captured["data"].time, 0.0)          # it really stepped the simulation


if __name__ == "__main__":
    unittest.main()
