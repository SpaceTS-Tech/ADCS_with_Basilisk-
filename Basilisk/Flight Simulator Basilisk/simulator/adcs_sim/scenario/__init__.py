"""Scenario assembly and execution.

=========================  =================================================
Module                     Contents
=========================  =================================================
:mod:`~.builder`           Instantiate every model, in order, and wire the
                           message graph.
:mod:`~.recorders`         Attach the message recorders a file-mode run needs.
:mod:`~.visualization`     Vizard interface, save file and live socket.
:mod:`~.spacecraft_model`  Custom spacecraft CAD model and its placement.
:mod:`~.camera`            Onboard camera views ("what the satellite sees").
:mod:`~.vizard_app`        Locate, launch and await the Vizard application.
:mod:`~.live`              Interactive keyboard-driven live-stream loop.
:mod:`~.runner`            Top-level ``run()`` for both modes.
:mod:`~.cli`               Command-line interface.
=========================  =================================================
"""

__all__ = ["builder", "camera", "cli", "live", "recorders", "runner",
           "spacecraft_model", "visualization", "vizard_app"]
