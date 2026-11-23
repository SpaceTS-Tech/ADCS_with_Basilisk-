"""AOCS connector: bind flight software to the Basilisk simulator by wiring.

Phase 1 -- port extraction:

* :func:`~aocs_connector.software_block.build_software_block` reads a
  Simulink Coder package (folder or DLL) into a block of ports.
* :func:`~aocs_connector.simulator_block.build_simulator_block` builds the
  Basilisk scenario and turns its message graph into a block of ports.

Both return :class:`~aocs_connector.ports.Block`.
"""

from aocs_connector.ports import Block, Port, block_from_dict

__all__ = ["Block", "Port", "block_from_dict"]
