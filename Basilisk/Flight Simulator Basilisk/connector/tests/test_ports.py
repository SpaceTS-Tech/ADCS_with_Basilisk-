"""Phase 1 checks: both extractors read the interfaces the bridge uses today.

Run from the ``connector`` folder::

    python -m unittest discover tests -v
"""

import sys
import unittest
from pathlib import Path

CONNECTOR_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONNECTOR_DIR))

from aocs_connector import c_header  # noqa: E402
from aocs_connector.ports import block_from_dict  # noqa: E402
from aocs_connector.simulator_block import (DEFAULT_SIMULATOR_DIR,  # noqa: E402
                                            build_simulator_block)
from aocs_connector.software_block import (build_software_block,  # noqa: E402
                                           check_library)

SOFTWARE_DIR = CONNECTOR_DIR.parents[1] / "ADCS_Software_v1_grt_rtw"
needs_package = unittest.skipUnless(SOFTWARE_DIR.is_dir(),
                                    "generated code not present")

SYNTHETIC = """
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_Bus_
#define DEFINED_TYPEDEF_FOR_Bus_
typedef struct {
  real_T vec[3];          /* nested bus member */
  uint16_T count;
} Bus;
#endif
#ifndef DEFINED_TYPEDEF_FOR_Bus_
typedef struct { int unused; } Bus;   /* guard duplicate: must be ignored */
#endif
typedef struct tag_RTM { const char_T *errorStatus; RTWLogInfo *log; } RTM;
typedef struct {
  Bus nav;                /* '<Root>/nav' */
  boolean_T flag;         /* '<Root>/flag' */
  int8_T table[2][4];
} ExtU_m_T;
extern ExtU_m_T m_U;
"""


class HeaderParserTest(unittest.TestCase):
    """The parser on a synthetic header, independent of the real package."""

    def test_flattens_buses_and_tolerates_unknown_types(self):
        types = c_header.HeaderTypes.from_texts([SYNTHETIC])
        self.assertIn("RTWLogInfo", types.opaque)

        leaves = {leaf.path: leaf
                  for leaf in c_header.flatten(types.ctype("ExtU_m_T"))}
        self.assertEqual(list(leaves), ["nav.vec", "nav.count", "flag", "table"])
        self.assertEqual((leaves["nav.vec"].dtype, leaves["nav.vec"].shape),
                         ("float64", (3,)))
        self.assertEqual(leaves["nav.count"].dtype, "uint16")
        self.assertEqual(leaves["flag"].dtype, "bool")
        self.assertEqual((leaves["table"].dtype, leaves["table"].shape),
                         ("int8", (2, 4)))
        # Native alignment: Bus is 8-aligned (24 + 2 -> 32), flag follows it.
        self.assertEqual(leaves["flag"].offset, 32)
        # The first Bus definition wins over the guarded duplicate.
        self.assertEqual([n for n, _ in types.ctype("Bus")._fields_],
                         ["vec", "count"])


@needs_package
class SoftwareBlockTest(unittest.TestCase):
    """The software block, on the real generated package."""

    @classmethod
    def setUpClass(cls):
        cls.block = build_software_block(SOFTWARE_DIR)

    def test_ports_match_hand_written_mirrors(self):
        """The headers give exactly the layout adcs_sim's mirrors encode."""
        sys.path.insert(0, str(DEFAULT_SIMULATOR_DIR))
        from adcs_sim.flightsoftware import structs

        for ports, mirror in ((self.block.inputs, structs.ADCSInputs),
                              (self.block.outputs, structs.ADCSOutputs)):
            self.assertEqual([p.id for p in ports],
                             [name for name, _ in mirror._fields_])
            for port in ports:
                self.assertEqual(port.extra["offset"],
                                 getattr(mirror, port.id).offset, port.id)
                self.assertEqual(port.extra["nbytes"],
                                 getattr(mirror, port.id).size, port.id)

    def test_library_agrees_with_headers(self):
        findings = check_library(self.block)
        self.assertFalse([f for f in findings if f[0] == "error"], findings)
        self.assertAlmostEqual(self.block.meta["sample_time"], 0.05)

    def test_dll_path_selects_the_same_model(self):
        via_dll = build_software_block(SOFTWARE_DIR / "ADCS_Software_v1.dll")
        self.assertEqual(via_dll.signature(), self.block.signature())

    def test_block_round_trips_through_json(self):
        restored = block_from_dict(self.block.to_dict())
        self.assertEqual(restored.signature(), self.block.signature())


@needs_package
class SimulatorBlockTest(unittest.TestCase):
    """The simulator block, built from adcs_sim."""

    @classmethod
    def setUpClass(cls):
        cls.block = build_simulator_block()

    def test_reproduces_the_bridge_wiring(self):
        bridge = set(map(tuple, self.block.meta["bridge_connections"]))
        self.assertIn(("SimpleNavigation.attOutMsg",
                       "ADCS_Software_v1.attNavInMsg"), bridge)
        self.assertIn(("ADCS_Software_v1.rwMotorTorqueOutMsg",
                       "bsk-Sat:ReactionWheelStateEffector.rwMotorCmdInMsg"),
                      bridge)

    def test_message_fields_become_ports(self):
        rw_cmd = self.block.port(
            "bsk-Sat:ReactionWheelStateEffector.rwMotorCmdInMsg.motorTorque")
        self.assertEqual(rw_cmd.direction, "in")
        self.assertEqual(rw_cmd.extra["wiring"]["status"], "flight_software")

        sigma = self.block.port("SimpleNavigation.attOutMsg.sigma_BN")
        self.assertEqual((sigma.direction, sigma.dtype, sigma.shape),
                         ("out", "float64", (3,)))

    def test_bridge_and_internal_wiring_are_left_out(self):
        self.assertFalse([p for p in self.block.ports
                          if p.id.startswith("ADCS_Software_v1.")])
        self.assertTrue(all(p.extra["wiring"]["status"] != "internal"
                            for p in self.block.inputs))


if __name__ == "__main__":
    unittest.main()
