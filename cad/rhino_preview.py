# venv: pyroki
"""Thin Rhino 8 adapter for :mod:`vurp_lower_plate`.

Run in Rhino's Python 3 editor, or import/call ``preview()`` from Grasshopper.
Only this file knows about Rhino or compas_rhino.
"""

import os
import sys

import Rhino
import System
import scriptcontext as sc
from compas.plugins import plugin_manager
from compas_rhino.conversions import brep_to_rhino
from compas_rhino.geometry import RhinoBrep
import compas_rhino.geometry.brep as rhino_brep_plugins


HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from vurp_lower_plate import PlateParameters, build_vurp_lower_plate, export_stls, set_brep_backend

# Some Rhino Python environments do not discover package entry points. Explicit
# registration makes the COMPAS Brep constructor/boolean pluggables available.
plugin_manager.register_module(rhino_brep_plugins)
set_brep_backend(RhinoBrep)


def _layer(name, color):
    index = sc.doc.Layers.FindByFullPath(name, Rhino.RhinoMath.UnsetIntIndex)
    if index >= 0:
        return index
    layer = Rhino.DocObjects.Layer()
    layer.Name = name
    layer.Color = color
    return sc.doc.Layers.Add(layer)


def preview(params=None, clear_previous=True):
    """Build with COMPAS, convert at the boundary, and add to Rhino."""
    params = params or PlateParameters()
    result = build_vurp_lower_plate(params)
    body_layer = _layer("VURP::Lower Plate", System.Drawing.Color.FromArgb(64, 135, 210))
    bezel_layer = _layer("VURP::Display Bezel", System.Drawing.Color.FromArgb(240, 155, 55))

    if clear_previous:
        for obj in list(sc.doc.Objects):
            if obj.Attributes.Name in (
                "VURP lower plate",
                "VURP display holder",
                "VURP display bezel",
            ):
                sc.doc.Objects.Delete(obj, True)

    guids = []
    for brep, name, layer_index in (
        (result.body, "VURP lower plate", body_layer),
        (result.display_holder, "VURP display holder", bezel_layer),
    ):
        geometry = brep_to_rhino(brep)
        attributes = Rhino.DocObjects.ObjectAttributes()
        attributes.Name = name
        attributes.LayerIndex = layer_index
        guids.append(sc.doc.Objects.AddBrep(geometry, attributes))
    sc.doc.Views.Redraw()
    return result, guids


def export_stl(output_directory, params=None):
    """Export separate plate/bezel STLs through COMPAS (never Rhino export)."""
    result = build_vurp_lower_plate(params or PlateParameters())
    return export_stls(result, os.path.abspath(output_directory))


if __name__ == "__main__":
    preview()
