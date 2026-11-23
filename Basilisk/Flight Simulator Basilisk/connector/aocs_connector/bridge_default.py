"""The wiring of today's hand-written bridge, as a link document.

Transcribed from ``adcs_sim/flightsoftware/controller.py``
(``_write_inputs`` / ``_read_outputs`` / ``_publish``), so the editor opens on
the binding the simulator actually uses, and phase 2 can prove the generic
runtime reproduces it.

Not expressible as wires, and therefore left out:

* ``Camerafeed`` / ``STR1`` -- filled from Vizard frames by
  ``CameraFeedReader``, not from a Basilisk message.
* Parameter writes at reset (mission command, gains, ``h_max``, fine-ADS
  flag) -- parameters, not signals.
* ``mtqVoltageOutMsg`` -- telemetry only, nothing subscribes to it.
"""

from aocs_connector.link import LinkDocument

NAV = "sim:SimpleNavigation"
RW = "sim:bsk-Sat:ReactionWheelStateEffector"
SPICE = "sim:SpiceInterfaceData"


def bridge_document(software_path, simulator_dir):
    doc = LinkDocument(
        software_path=str(software_path),
        simulator={"entry": "adcs_sim.scenario.builder:build_simulation",
                   "simulator_dir": str(simulator_dir)})

    # -- simulator -> software ---------------------------------------------
    q = doc.add_node("mrp_to_quat", (420, 40), node_id="n1")
    doc.connect(f"{NAV}.attOutMsg.sigma_BN", f"{q}:sigma")
    doc.connect(f"{q}:q", "sw:q_BI")

    doc.connect(f"{NAV}.attOutMsg.omega_BN_B", "sw:omega")

    mag = doc.add_node("rotate_to_body", (420, 160), node_id="n2")
    doc.connect(f"{NAV}.attOutMsg.sigma_BN", f"{mag}:sigma_BN")
    doc.connect("sim:CenteredDipoleMagneticField.envOutMsgs[0].magField_N",
                f"{mag}:v_N")
    doc.connect(f"{mag}:v_B", "sw:B")

    wheels = doc.add_node("select", (420, 290), {"indices": "0,1,2"},
                          node_id="n3")
    doc.connect(f"{RW}.rwSpeedOutMsg.wheelSpeeds", f"{wheels}:u")
    doc.connect(f"{wheels}:y", "sw:RW_omega")

    doc.connect(f"{NAV}.transOutMsg.v_BN_N", "sw:v_sat_i")
    doc.connect(f"{NAV}.transOutMsg.r_BN_N", "sw:r_sat_i")

    target = doc.add_node(
        "python_function", (420, 390),
        {"inputs": 1,
         "function": "adcs_sim.geometry.ground_target:position_inertial"},
        node_id="n4")
    doc.nodes[target].label = "Ground target (ECI)"
    doc.connect(f"{SPICE}.planetStateOutMsgs[0].J20002Pfix", f"{target}:u1")
    doc.connect(f"{target}:y", "sw:r_p_i")

    doc.connect(f"{SPICE}.planetStateOutMsgs[1].VelocityVector", "sw:v_sun_i")
    doc.connect(f"{SPICE}.planetStateOutMsgs[1].PositionVector", "sw:r_sun_i")

    # -- software -> simulator ---------------------------------------------
    rw_limit = doc.add_node(
        "python_function", (1180, 40),
        {"inputs": 1,
         "function": "adcs_sim.models.reaction_wheels:limit_motor_torque"},
        node_id="n5")
    doc.nodes[rw_limit].label = "RW torque limit"
    doc.connect("sw:RW_tau", f"{rw_limit}:u1")
    doc.connect(f"{rw_limit}:y", f"{RW}.rwMotorCmdInMsg.motorTorque")

    mtq_limit = doc.add_node(
        "python_function", (1180, 170),
        {"inputs": 1, "function": "adcs_sim.models.magnetorquers:limit_voltage"},
        node_id="n6")
    doc.nodes[mtq_limit].label = "MTQ voltage limit"
    dipole = doc.add_node(
        "python_function", (1400, 170),
        {"inputs": 1,
         "function": "adcs_sim.models.magnetorquers:voltage_to_dipole"},
        node_id="n7")
    doc.nodes[dipole].label = "Voltage → dipole"
    doc.connect("sw:MTQ_tau", f"{mtq_limit}:u1")
    doc.connect(f"{mtq_limit}:y", f"{dipole}:u1")
    doc.connect(f"{dipole}:y", "sim:MTQ_effector.mtbCmdInMsg.mtbDipoleCmds")

    thr_limit = doc.add_node(
        "python_function", (1180, 300),
        {"inputs": 1, "function": "adcs_sim.models.thrusters:limit_body_torque"},
        node_id="n8")
    doc.nodes[thr_limit].label = "THR torque limit"
    doc.connect("sw:THR_tau", f"{thr_limit}:u1")
    doc.connect(f"{thr_limit}:y",
                "sim:Thruster_torque_effector.cmdTorqueInMsg.torqueRequestBody")
    return doc
