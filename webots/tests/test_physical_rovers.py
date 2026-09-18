"""Guardas estructurales: la simulación no puede volver a ser una animación."""

import unittest
from pathlib import Path


WEBOTS = Path(__file__).resolve().parents[1]
WORLD = WEBOTS / "worlds" / "vision_rover_challenge.wbt"
SUPERVISOR = WEBOTS / "controllers" / "vision_supervisor" / "vision_supervisor.py"
AGENT = WEBOTS / "controllers" / "rover_agent" / "rover_agent.py"


class PhysicalRoversTest(unittest.TestCase):
    def test_each_rover_has_two_powered_physical_wheels(self):
        text = WORLD.read_text(encoding="utf-8")
        self.assertNotIn('coordinateSystem "EUN"', text)
        self.assertIn("DirectionalLight", text)
        self.assertEqual(text.count('name "left_wheel_motor"'), 2)
        self.assertEqual(text.count('name "right_wheel_motor"'), 2)
        self.assertGreaterEqual(text.count("HingeJoint {"), 4)
        self.assertGreaterEqual(text.count("boundingObject Cylinder"), 4)

    def test_agent_drives_motors_not_supervisor_teleportation(self):
        agent = AGENT.read_text(encoding="utf-8")
        supervisor = SUPERVISOR.read_text(encoding="utf-8")
        self.assertIn("left_motor.setVelocity", agent)
        self.assertIn("right_motor.setVelocity", agent)
        self.assertIn("VRC_ACTUATOR_TEST", agent)
        self.assertNotIn("def update_rover", supervisor)
        # Restablecer una ronda puede posicionar objetos una vez; durante la
        # ejecución no debe existir una función que los avance a mano.
        self.assertEqual(supervisor.count('cube["t"].setSFVec3f(['), 1)


if __name__ == "__main__":
    unittest.main()
