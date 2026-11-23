"""Conversion blocks and link files.

Run from the ``connector`` folder::

    python -m unittest discover tests -v
"""

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

CONNECTOR_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONNECTOR_DIR))

from aocs_connector import conversions  # noqa: E402
from aocs_connector.link import LinkDocument, validate  # noqa: E402

LINK = CONNECTOR_DIR / "links" / "ADCS_Software_v1_bridge.aocslink"


class AttitudeMathTest(unittest.TestCase):
    """The numpy conversions match Basilisk's RigidBodyKinematics."""

    def test_matches_basilisk(self):
        from Basilisk.utilities import RigidBodyKinematics as rbk
        sigma = np.array([0.1, -0.3, 0.25])
        np.testing.assert_allclose(conversions.mrp_to_quaternion(sigma),
                                   rbk.MRP2EP(sigma))
        np.testing.assert_allclose(conversions.mrp_to_dcm(sigma),
                                   rbk.MRP2C(sigma))
        np.testing.assert_allclose(
            conversions.quaternion_to_mrp(rbk.MRP2EP(sigma)), sigma)

    def test_infer_outputs(self):
        outputs, _ = conversions.infer_outputs(
            "place", {"length": 36, "offset": 0}, [("float64", (3,))])
        self.assertEqual(outputs, [("float64", (36,))])
        outputs, error = conversions.infer_outputs(
            "select", {"indices": "0,9"}, [("float64", (3,))])
        self.assertIsNone(outputs)
        self.assertIn("IndexError", error)


class LinkFileTest(unittest.TestCase):

    def test_round_trip_keeps_paths_and_wires(self):
        doc = LinkDocument.load(LINK)
        with tempfile.TemporaryDirectory() as folder:
            copy = Path(folder) / "copy.aocslink"
            doc.save(copy)
            again = LinkDocument.load(copy)
        self.assertEqual(again.software_path, doc.software_path)
        self.assertEqual(again.simulator, doc.simulator)
        self.assertEqual(again.wires, doc.wires)
        self.assertEqual(again.nodes.keys(), doc.nodes.keys())

    def test_connect_replaces_the_old_wire_into_an_input(self):
        doc = LinkDocument()
        doc.connect("sim:a.x", "sw:q")
        doc.connect("sim:b.y", "sw:q")
        self.assertEqual(doc.wires, [("sim:b.y", "sw:q")])

    def test_bridge_wiring_validates_against_live_interfaces(self):
        from aocs_connector.simulator_block import build_simulator_block
        from aocs_connector.software_block import build_software_block

        doc = LinkDocument.load(LINK)
        software = build_software_block(doc.software_path)
        simulator = build_simulator_block(
            simulator_dir=doc.simulator["simulator_dir"])
        report = validate(doc, software, simulator)
        self.assertEqual(report.counts()["error"], 0,
                         [i.message for i in report.issues])
        # Only the camera inports are left open: they come from Vizard.
        self.assertEqual(
            sorted(i.ref for i in report.issues if i.level == "warning"),
            ["sw:Camerafeed", "sw:STR1"])

    def test_removed_port_becomes_a_broken_wire(self):
        doc = LinkDocument()
        doc.connect("sw:no_such_output", "sw:also_missing")
        report = validate(doc)
        self.assertEqual(report.counts()["error"], 1)


if __name__ == "__main__":
    unittest.main()
