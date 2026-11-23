"""Build the star catalog the simulated star tracker camera renders from.

Reads the Hipparcos main catalogue export (VizieR ``I/239/hip_main``, all
stars with V <= 7) and writes ``adcs_sim/data/hipparcos_j2000.npz``:

==========  =======  =================================================
``hip``     int32    Hipparcos number
``vector``  float64  unit direction, ICRS axes (= J2000 to ~0.02 arcsec)
``vmag``    float32  Johnson V magnitude
==========  =======  =================================================

Positions are **epoch J2000.0**: VizieR's ``_RA.icrs``/``_DE.icrs`` columns,
which apply each star's proper motion from the catalogue epoch J1991.25.  The
dozen entries with no astrometric solution fall back to the sexagesimal
``RAhms``/``DEdms`` positions.

This is deliberately *not* the catalogue compiled into the flight software
(1102 Yale Bright Star Catalogue stars with instrument magnitudes): rendering
from an independent catalogue means the simulator does not hand the star
tracker exactly the sky it expects -- faint distractors, magnitude
differences and double stars included.

Usage::

    python tools/build_star_catalog.py [path/to/hipparcos_full.csv]
"""

import csv
import sys
from pathlib import Path

import numpy as np

SIMULATOR = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (SIMULATOR.parents[3] / "Star Trcaker" / "Catalogs"
                  / "hipparcos_full.csv")
OUTPUT = SIMULATOR / "adcs_sim" / "data" / "hipparcos_j2000.npz"


def _sexagesimal(text, hours):
    sign = -1.0 if text.strip().startswith("-") else 1.0
    a, b, c = (abs(float(part)) for part in text.split())
    value = sign * (a + b / 60.0 + c / 3600.0)
    return value * 15.0 if hours else value


def load(path):
    hip, ra, dec, vmag = [], [], [], []
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if not row["Vmag"].strip():
                continue
            if row["_RA.icrs"].strip():
                ra_deg = float(row["_RA.icrs"])
                dec_deg = float(row["_DE.icrs"])
            else:
                ra_deg = _sexagesimal(row["RAhms"], hours=True)
                dec_deg = _sexagesimal(row["DEdms"], hours=False)
            hip.append(int(row["HIP"]))
            ra.append(ra_deg)
            dec.append(dec_deg)
            vmag.append(float(row["Vmag"]))
    ra, dec = np.radians(ra), np.radians(dec)
    vectors = np.column_stack((np.cos(dec) * np.cos(ra),
                               np.cos(dec) * np.sin(ra),
                               np.sin(dec)))
    return (np.asarray(hip, dtype=np.int32), vectors,
            np.asarray(vmag, dtype=np.float32))


def main(argv):
    source = Path(argv[1]) if len(argv) > 1 else DEFAULT_SOURCE
    hip, vectors, vmag = load(source)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUTPUT, hip=hip, vector=vectors, vmag=vmag)
    print(f"{len(hip)} stars, V {vmag.min():.2f} .. {vmag.max():.2f}  ->  "
          f"{OUTPUT} ({OUTPUT.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main(sys.argv)
