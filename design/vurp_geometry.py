"""Grasshopper side-car loader for the canonical VURP geometry module.

This file sits beside ``plate-design.gh`` so ``compas_rhino.DevTools`` watches
it. ``load_core`` also executes the canonical module fresh on every solution,
so edits in ``../cad/vurp_lower_plate.py`` are never hidden by ``sys.modules``.
"""

import importlib.util
import os
import sys


CORE_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "cad", "vurp_lower_plate.py")
)
MODULE_NAME = "_vurp_lower_plate_live"


def load_core():
    spec = importlib.util.spec_from_file_location(MODULE_NAME, CORE_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Cannot load VURP geometry module: {}".format(CORE_PATH))
    module = importlib.util.module_from_spec(spec)
    # Dataclasses resolve annotations through sys.modules while executing.
    sys.modules[MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module
