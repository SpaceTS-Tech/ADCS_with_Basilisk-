"""Interactive live-stream loop driven by Vizard keyboard input.

Inventory item **C4**.  The simulation is stepped one controller period at a
time so that a key press is acted on within one step.

===============  ==========================================================
Key              Action
===============  ==========================================================
``0``-``9``      Select mission command 0-9.
``a`` / ``b``    Select mission command 10 / 11.
``p``            Toggle pause (the Vizard view keeps updating while paused).
``q``            Terminate Vizard and return.
===============  ==========================================================
"""

import os
import time

from Basilisk.utilities import macros, vizSupport

from adcs_sim.config import actuators as act_cfg
from adcs_sim.config import simulation as sim_cfg
from adcs_sim.config import visualization as viz_cfg
from adcs_sim.scenario.live_plot import create as create_live_plot

#: Log every change in the keyboard string Vizard sends back.  Set
#: ``ADCS_KEY_DEBUG=1`` to diagnose a run that pauses or quits on its own --
#: it distinguishes a real key press from a spurious or malformed message.
#: Off by default and read-only; it cannot change the loop's behaviour.
KEY_DEBUG = os.environ.get("ADCS_KEY_DEBUG") == "1"


def mission_key_map():
    """Return the key -> mission-command mapping."""
    keys = {str(value): value for value in range(10)}
    keys.update({"a": 10, "b": 11})
    return keys


def _power_sample(array):
    """Read the solar array's current output, or ``None`` when disabled.

    Four message reads per step.  Telemetry only: nothing read here is fed
    back into the controller.
    """
    if array is None:
        return None
    return {
        "total_w": array.total_power_w(),
        "panels_w": array.panel_powers_w(),
        "energy_wh": array.energy_wh(),
        "illumination": array.illumination_factor(),
    }


def _banner(controller):
    return [
        f"Live Vizard stream connected on "
        f"{viz_cfg.VIZ_SOCKET_PROTOCOL}://{viz_cfg.VIZ_SOCKET_ADDRESS}:"
        f"{viz_cfg.VIZ_SOCKET_PORT}",
        "Mission keys: 0-9 => commands 0-9, A => 10, B => 11",
        "Controls: P => pause/resume, Q => quit",
        f"Starting mission command {controller.current_mission_cmd}",
    ]


def run_live_stream(bundle, viz, clock_sync, live_plot=True):
    """Step the simulation indefinitely, servicing Vizard key presses.

    ``live_plot`` opens the ACS-mode strip chart; ``--no-live-plot`` clears it.
    """
    sim = bundle.sim
    controller = bundle.controller
    step_nanos = macros.sec2nano(sim_cfg.CONTROLLER_STEP_SECONDS)
    mission_keys = mission_key_map()

    stop_time = 0
    previous_keys = ""

    for line in _banner(controller):
        print(line)

    plot = create_live_plot() if live_plot else None
    if plot is not None and plot.enabled:
        print("Live ACS-mode plot open (close the window to stop plotting; "
              "the simulation keeps running).")

    try:
        while True:
            if vizSupport.pauseFlag:
                # Keep the Vizard view alive and reset the pacing clock so the
                # simulation does not sprint to catch up on resume.
                viz.UpdateState(stop_time)
                clock_sync.Reset(0)
                time.sleep(viz_cfg.LIVE_PAUSE_POLL_SECONDS)
            else:
                stop_time += step_nanos
                sim.ConfigureStopTime(stop_time)
                sim.ExecuteSimulation()

                if plot is not None:
                    # actual_acs, not the commanded mission mode: the manager
                    # can sit in DESAT/DETUMBLE/SAFE underneath any command,
                    # and that difference is the whole point of the chart.
                    wheel_speeds = controller.rwSpeedsInMsg().wheelSpeeds
                    plot.update(
                        stop_time * macros.NANO2SEC,
                        controller.library.block_signals.actual_acs,
                        [act_cfg.CONTROLLER_I_RW * wheel_speeds[i]
                         for i in range(3)],
                        mission_cmd=controller.current_mission_cmd,
                        power=_power_sample(bundle.power),
                        q_est=list(controller.library.outputs.q_BI_est),
                        q_true=list(controller.library.inputs.q_BI))

            keys = viz.userInputMsg.read().keyboardInput.lower()
            if KEY_DEBUG and keys != previous_keys:
                print(f"t={stop_time * macros.NANO2SEC:8.2f} s  keyboardInput="
                      f"{keys!r}  new={sorted(set(keys) - set(previous_keys))}",
                      flush=True)
            for key in set(keys) - set(previous_keys):
                if key in mission_keys:
                    controller.set_mission_command(mission_keys[key])
                    print(f"t={stop_time * macros.NANO2SEC:.2f} s: "
                          f"mission command -> {mission_keys[key]}")
                elif key == "p":
                    vizSupport.pauseFlag = not vizSupport.pauseFlag
                    print("Simulation paused" if vizSupport.pauseFlag
                          else "Simulation resumed")
                elif key == "q":
                    raise KeyboardInterrupt
            previous_keys = keys
    except KeyboardInterrupt:
        print("Stopping live simulation.")
        viz.liveSettings.terminateVizard = True
        viz.UpdateState(stop_time)
    finally:
        if plot is not None:
            plot.close()
