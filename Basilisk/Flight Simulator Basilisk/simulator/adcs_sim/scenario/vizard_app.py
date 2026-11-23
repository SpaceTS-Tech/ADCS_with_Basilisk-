"""Launch the Vizard viewer and wait for its socket, for ``--mode live``.

Who connects to whom
--------------------
Basilisk's ``vizInterface`` is the **client**: in live mode it connects out to
``reqComAddress:reqPortNumber`` and blocks, logging
``Waiting for Vizard at tcp://127.0.0.1:5556``.  **Vizard is the server** and
must be listening on that port before the stream can start.

Driving Vizard from the command line
------------------------------------
Vizard accepts these flags (extracted from its ``Assembly-CSharp.dll``):
``-directComm``, ``-noDisplay``, ``-opNav``, ``-opNavMode``, ``-loadFile``,
``-saveMsgFile``, ``-saveMetrics``.

``-directComm`` **takes the socket address as its next argument**::

    Vizard.exe -directComm tcp://127.0.0.1:5556

Given that, Vizard opens the socket itself within a couple of seconds and logs
``Waiting for messages to load`` -- no UI interaction is needed.

Passing ``-directComm`` *alone* is a trap: Vizard throws
``IndexOutOfRangeException`` in ``StartUpScreenManager.Start()`` (it reads the
address at ``args[i + 1]``), never opens the socket, and sits on its start
screen.  Both behaviours were confirmed empirically against Vizard's
``Player.log``.

Start-up race -- why nothing is awaited by default
--------------------------------------------------
Both sides wait for each other, but only one waits forever:

* **Basilisk** blocks indefinitely at the end of ``InitializeSimulation``,
  logging ``Waiting for Vizard at tcp://...``.
* **Vizard** gives up, logging ``Timed out waiting for messages to load``.

So Basilisk must be the one kept waiting.  Blocking on Vizard's socket *before*
``InitializeSimulation`` serialises the two start-ups -- Vizard's timer begins
while Basilisk is still resetting its models -- and was observed to lose that
race on a loaded machine (Vizard took 19.8 s to boot, then timed out).

Launching Vizard and then going straight into ``InitializeSimulation`` overlaps
the two, so Basilisk is normally already waiting when Vizard's socket appears.
That is the default; :func:`VizardSession.wait_until_listening` is retained for
diagnostics and is gated behind
:data:`~adcs_sim.config.visualization.VIZARD_WAIT_FOR_SOCKET`.
"""

import atexit
import os
import socket
import subprocess
import threading
import time
from pathlib import Path

from adcs_sim.config import visualization as viz_cfg

#: Windows creation flag that keeps helper consoles from flashing up.
_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


class VizardNotFoundError(RuntimeError):
    """Raised when no Vizard executable could be located."""


def _explicit_path(explicit=None):
    """Return a deliberately specified path, or ``None`` if none was given.

    An explicit choice is honoured strictly: if the user names a path via
    ``--vizard-exe``, the ``VIZARD_EXECUTABLE`` environment variable or the
    config constant, a wrong path is an error rather than a silent fall-through
    to some other Vizard on the machine.
    """
    for source, value in (
        ("--vizard-exe", explicit),
        (f"${viz_cfg.VIZARD_EXECUTABLE_ENV_VAR}",
         os.environ.get(viz_cfg.VIZARD_EXECUTABLE_ENV_VAR)),
        ("config.visualization.VIZARD_EXECUTABLE", viz_cfg.VIZARD_EXECUTABLE),
    ):
        if not value:
            continue
        path = Path(os.path.expandvars(os.path.expanduser(str(value))))
        if not path.is_file():
            raise VizardNotFoundError(
                f"Vizard executable from {source} does not exist: {path}")
        return path
    return None


def _searched_paths():
    """Yield candidate executable paths from the configured search globs."""
    for pattern in viz_cfg.VIZARD_SEARCH_GLOBS:
        expanded = os.path.expandvars(os.path.expanduser(pattern))
        # Anchored patterns still need globbing for the drive-letter wildcards.
        path = Path(expanded)
        if any(character in expanded for character in "*?"):
            root = Path(path.anchor or ".")
            relative = str(path.relative_to(path.anchor)) if path.anchor else str(path)
            try:
                yield from sorted(root.glob(relative))
            except (OSError, ValueError):
                continue
        else:
            yield path


def find_executable(explicit=None):
    """Return the Vizard executable path.

    Resolution order: explicit argument (``--vizard-exe``), the
    ``VIZARD_EXECUTABLE`` environment variable,
    :data:`~adcs_sim.config.visualization.VIZARD_EXECUTABLE`, then the
    configured search globs.  The first three are strict -- a named path that
    does not exist raises instead of falling through to the search.

    Raises
    ------
    VizardNotFoundError
        If a named path does not exist, or nothing was found at all.
    """
    named = _explicit_path(explicit)
    if named is not None:
        return named

    for candidate in _searched_paths():
        if candidate.is_file():
            return candidate
    searched = "\n  ".join(str(p) for p in viz_cfg.VIZARD_SEARCH_GLOBS)
    raise VizardNotFoundError(
        "Could not find the Vizard executable. Set VIZARD_EXECUTABLE in "
        "adcs_sim/config/visualization.py, export the VIZARD_EXECUTABLE "
        f"environment variable, or pass --vizard-exe.\nSearched:\n  {searched}")


def port_is_listening(host, port, timeout=0.25):
    """Return True if something accepts TCP connections at ``host:port``.

    A bare connect-and-close probe.  ZeroMQ tolerates a peer that disconnects
    immediately, so this does not disturb a Vizard that is already up.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.settimeout(timeout)
            return probe.connect_ex((host, int(port))) == 0
    except OSError:
        return False


def listening_pid(port):
    """Return the PID listening on ``port``, or ``None`` if nothing is.

    Parsed from ``netstat -ano`` so no third-party dependency is needed.  Used
    to clear a Vizard orphaned by a previous run: such a process still holds
    the port, so a new Vizard cannot bind it, but it has already timed out and
    will never serve the stream.
    """
    if os.name != "nt":
        return None
    try:
        completed = subprocess.run(
            ["netstat", "-a", "-n", "-o", "-p", "TCP"],
            capture_output=True, text=True, timeout=15,
            creationflags=_NO_WINDOW)
    except (OSError, subprocess.SubprocessError):
        return None

    suffix = f":{port}"
    for line in completed.stdout.splitlines():
        fields = line.split()
        if (len(fields) >= 5 and fields[3].upper() == "LISTENING"
                and fields[1].endswith(suffix)):
            try:
                return int(fields[4])
            except ValueError:
                return None
    return None


def terminate_pid(pid):
    """Force-kill ``pid`` and its children.  Best effort; errors are ignored."""
    if pid is None:
        return False
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(pid), "/F", "/T"],
                           capture_output=True, timeout=15,
                           creationflags=_NO_WINDOW)
        else:
            os.kill(pid, 9)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def wait_for_port_release(host, port, timeout=10.0):
    """Block until nothing is listening on the port.  True if it came free."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not port_is_listening(host, port):
            return True
        time.sleep(0.25)
    return not port_is_listening(host, port)


class VizardSession:
    """Own the lifetime of an auto-launched Vizard process.

    Usable as a context manager.  A Vizard already listening on the target port
    is replaced rather than adopted unless
    :data:`~adcs_sim.config.visualization.VIZARD_ADOPT_EXISTING` says otherwise
    -- see that setting for why adopting is the wrong default.
    """

    def __init__(self, host, port, executable=None, launch_args=None,
                 protocol=None):
        self.host = host
        self.port = str(port)
        self.protocol = protocol or viz_cfg.VIZ_SOCKET_PROTOCOL
        self._explicit_executable = executable
        self._launch_arg_template = (tuple(viz_cfg.VIZARD_LAUNCH_ARGS)
                                     if launch_args is None
                                     else tuple(launch_args))
        self.executable = None
        self.process = None
        self.adopted_existing = False
        self._stopped = False

    @property
    def socket_address(self):
        """The address Basilisk connects to and Vizard is told to open."""
        return f"{self.protocol}://{self.host}:{self.port}"

    @property
    def launch_args(self):
        """Launch arguments with ``{socket}`` and friends substituted."""
        return [argument.format(socket=self.socket_address,
                                protocol=self.protocol,
                                address=self.host,
                                port=self.port)
                for argument in self._launch_arg_template]

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        """Locate and launch Vizard, clearing anything stale on the port."""
        if port_is_listening(self.host, self.port):
            if viz_cfg.VIZARD_ADOPT_EXISTING:
                self.adopted_existing = True
                print(f"Vizard already listening on "
                      f"tcp://{self.host}:{self.port} -- using it.")
                return self
            self._clear_stale_listener()

        self._launch()
        # Even a hard-killed console leaves the orphan behind otherwise, and
        # that orphan is what poisons the *next* run.
        atexit.register(self.stop)
        return self

    def _clear_stale_listener(self):
        """Terminate whatever already holds the port so a fresh Vizard can bind."""
        pid = listening_pid(self.port)
        print(f"Something is already listening on tcp://{self.host}:{self.port}"
              f"{'' if pid is None else f' (pid {pid})'} -- most likely a Vizard "
              "orphaned by an earlier run. Replacing it.")
        if pid is not None:
            terminate_pid(pid)
        if not wait_for_port_release(self.host, self.port):
            print(f"  Port {self.port} is still held; the new Vizard may fail "
                  "to bind. Close any running Vizard and retry.")

    def _launch(self):
        """Start the Vizard process (no port checks; see :meth:`start`)."""
        self.executable = find_executable(self._explicit_executable)
        command = [str(self.executable), *self.launch_args]
        print(f"Launching Vizard: {' '.join(command)}")
        # Start Vizard detached from our console so its own logging does not
        # interleave with the simulation's stdout.
        creation_flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        self.process = subprocess.Popen(
            command,
            cwd=str(self.executable.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=creation_flags,
        )

    # -- deadlock recovery -------------------------------------------------

    def start_watchdog(self):
        """Relaunch Vizard until the stream connects.

        ``InitializeSimulation`` blocks inside C++ with no way to interrupt it,
        so recovery has to happen on another thread.  Returns the
        :class:`threading.Event` to set once the stream is up; the watchdog
        stops as soon as it is set.

        This is what turns the classic failure -- Vizard times out, Basilisk
        waits on a dead port forever -- into a retry instead of a hang.
        """
        connected = threading.Event()
        if not viz_cfg.VIZARD_WATCHDOG_ENABLED or self.adopted_existing:
            connected.set()
            return connected

        thread = threading.Thread(target=self._watch, args=(connected,),
                                  name="vizard-watchdog", daemon=True)
        thread.start()
        return connected

    def _watch(self, connected):
        """Watchdog body.  Runs until ``connected`` is set or attempts run out."""
        attempts_left = viz_cfg.VIZARD_RELAUNCH_ATTEMPTS
        last_launch = time.monotonic()
        saw_socket = False

        while not connected.wait(viz_cfg.VIZARD_POLL_SECONDS):
            listening = port_is_listening(self.host, self.port)
            saw_socket = saw_socket or listening

            if self.process is not None and self.process.poll() is not None:
                reason = f"Vizard exited (code {self.process.returncode})"
            elif saw_socket and not listening:
                reason = "Vizard closed its socket (it timed out waiting)"
            elif time.monotonic() - last_launch > viz_cfg.VIZARD_RELAUNCH_AFTER_SECONDS:
                reason = (f"no connection "
                          f"{viz_cfg.VIZARD_RELAUNCH_AFTER_SECONDS:g} s after launch")
            else:
                continue

            if attempts_left <= 0:
                print(f"\n[vizard] {reason}, and the relaunch limit is reached.")
                self._print_manual_fallback()
                return

            attempts_left -= 1
            print(f"\n[vizard] {reason}; relaunching "
                  f"({viz_cfg.VIZARD_RELAUNCH_ATTEMPTS - attempts_left}/"
                  f"{viz_cfg.VIZARD_RELAUNCH_ATTEMPTS}). "
                  "Basilisk is still waiting, so the run continues.")
            self._relaunch()
            last_launch = time.monotonic()
            saw_socket = False

    def _relaunch(self):
        """Kill the current Vizard (and any port squatter) and start a new one."""
        if self.process is not None and self.process.poll() is None:
            terminate_pid(self.process.pid)
        if port_is_listening(self.host, self.port):
            terminate_pid(listening_pid(self.port))
            wait_for_port_release(self.host, self.port)
        try:
            self._launch()
        except VizardNotFoundError as error:
            print(f"[vizard] Relaunch failed: {error}")

    def wait_until_listening(self, timeout=None):
        """Block until Vizard's socket is up, or the timeout expires.

        Returns
        -------
        bool
            True if the port came up.  On timeout this returns False rather
            than raising: Basilisk keeps waiting and retrying on its own, so a
            slow start is recoverable and must not abort the run.
        """
        if self.adopted_existing:
            return True
        timeout = (viz_cfg.VIZARD_STARTUP_TIMEOUT_SECONDS
                   if timeout is None else timeout)

        started = time.monotonic()
        while time.monotonic() - started < timeout:
            if port_is_listening(self.host, self.port):
                print(f"Vizard is listening on {self.socket_address} after "
                      f"{time.monotonic() - started:.1f} s -- starting the "
                      "stream.")
                return True
            if self.process is not None and self.process.poll() is not None:
                print(f"Vizard exited before opening the socket "
                      f"(exit code {self.process.returncode}). "
                      f"Check {_player_log_path()}")
                return False
            time.sleep(viz_cfg.VIZARD_POLL_SECONDS)

        print(f"Timed out after {timeout:g} s waiting for Vizard to open "
              f"{self.socket_address}.")
        self._print_manual_fallback()
        return False

    def stop(self):
        """Terminate an auto-launched Vizard, if configured to do so.

        Idempotent: it is reached both from the normal ``finally`` path and
        from the :mod:`atexit` hook registered in :meth:`start`.
        """
        if self._stopped or self.process is None or self.adopted_existing:
            return
        self._stopped = True
        if not viz_cfg.VIZARD_TERMINATE_ON_EXIT:
            print("Leaving Vizard running (VIZARD_TERMINATE_ON_EXIT is False).")
            return
        if self.process.poll() is not None:
            return
        print("Closing Vizard.")
        self.process.terminate()
        try:
            self.process.wait(timeout=viz_cfg.VIZARD_SHUTDOWN_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            # terminate() does not take Vizard's children with it; a survivor
            # would keep the port and poison the next run.
            terminate_pid(self.process.pid)

    # -- context manager ---------------------------------------------------

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, traceback):
        self.stop()
        return False

    # -- helpers -----------------------------------------------------------

    def _print_manual_fallback(self):
        """Explain how to connect by hand when the auto-launch fell short."""
        print("Basilisk will keep retrying, so you can still connect manually:")
        print(f"  1. In Vizard, choose direct communication / two-way (RxTx)")
        print(f"  2. Socket address: {self.socket_address}")
        print(f"  3. Display mode: Live Display, then press connect/start")
        print(f"Vizard's own log: {_player_log_path()}")


def _player_log_path():
    """Return Vizard's Unity player log path, for troubleshooting hints."""
    return (Path(os.path.expandvars("%USERPROFILE%")) / "AppData" / "LocalLow"
            / "Vizard" / "Vizard" / "Player.log")
