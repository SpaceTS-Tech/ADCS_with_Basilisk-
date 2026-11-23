"""Matplotlib backend selection.

:func:`select` must run **before** ``matplotlib.pyplot`` is first imported
anywhere in the process, which is why the plotting module is imported lazily by
:func:`adcs_sim.analysis.write_analysis`.
"""

import matplotlib

#: Headless backend used for saving figures to disk.
FILE_BACKEND = "Agg"


def select(interactive=False):
    """Choose the backend for this run.

    Parameters
    ----------
    interactive : bool
        Leave the environment's default (window-capable) backend in place so
        ``plt.show()`` actually displays something.  Otherwise force
        :data:`FILE_BACKEND`, which saves figures without needing a display.

    Returns
    -------
    str
        The backend now in effect.
    """
    if not interactive:
        matplotlib.use(FILE_BACKEND)
    return matplotlib.get_backend()
