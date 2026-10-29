#!/usr/bin/env python3
"""Tessellate the AG-160-95 STEP assembly's main shell and flange for the viewer.

Run from the repository root with:
    uv run --with cadquery python data_viewer/scripts/export_ag160_meshes.py

The STEP uses millimetres and keeps the assembly coordinates in the exported STL.
The viewer maps the CAD flange origin and axes into the Nova Link6 frame.
"""

from __future__ import annotations

from pathlib import Path

import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool


ROOT = Path(__file__).resolve().parents[2]
STEP_FILE = ROOT / "AG-160-95-W-S_DH-Robotics-.step"
OUTPUT_DIR = ROOT / "data_viewer/public/models/ag160"


def close(actual: float, expected: float, tolerance: float = 0.5) -> bool:
    return abs(actual - expected) <= tolerance


def dimensions(shape: cq.Shape) -> tuple[float, float, float]:
    box = shape.BoundingBox()
    return box.xlen, box.ylen, box.zlen


def find_fingertip_shape() -> cq.Shape:
    app = XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), document)
    reader = STEPCAFControl_Reader()
    if reader.ReadFile(str(STEP_FILE)).name != "IFSelect_RetDone" or not reader.Transfer(document):
        raise RuntimeError("Could not read STEP assembly hierarchy")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)

    def label_name(label: TDF_Label) -> str:
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""

    def search(label: TDF_Label, depth: int = 0) -> cq.Shape | None:
        if depth > 12:
            return None
        components = TDF_LabelSequence()
        shape_tool.GetComponents_s(label, components, False)
        for index in range(1, components.Length() + 1):
            component = components.Value(index)
            referred = TDF_Label()
            if not shape_tool.GetReferredShape_s(component, referred):
                continue
            name = label_name(referred)
            if "FINGER_FINGERTIP" in name:
                return cq.Shape.cast(shape_tool.GetShape_s(referred))
            if shape_tool.IsAssembly_s(referred):
                found = search(referred, depth + 1)
                if found is not None:
                    return found
        return None

    for index in range(1, roots.Length() + 1):
        shape = search(roots.Value(index))
        if shape is not None:
            return shape
    raise RuntimeError("Could not find FINGER_FINGERTIP in STEP assembly")


def main() -> None:
    if not STEP_FILE.is_file():
        raise FileNotFoundError(f"Missing AG-160-95 STEP file: {STEP_FILE}")

    imported = cq.importers.importStep(str(STEP_FILE)).val()

    # A.STEP is a shell-based surface model rather than a watertight solid.
    # It is the sole high-face-count shell in the assembled STEP import.
    main_shells = [shell for shell in imported.Shells() if len(shell.Faces()) > 1000]
    if len(main_shells) != 1:
        raise RuntimeError(f"Expected one A.STEP shell, found {len(main_shells)}")
    main_shell = main_shells[0]
    main_dims = dimensions(main_shell)
    if not all(close(value, expected) for value, expected in zip(main_dims, (93.0, 129.5, 52.3))):
        raise RuntimeError(f"Unexpected A.STEP shell bounds: {main_dims}")

    flange_candidates = []
    for solid in imported.Solids():
        dims = dimensions(solid)
        if all(close(value, expected) for value, expected in zip(dims, (93.0, 18.2, 67.0))):
            flange_candidates.append(solid)
    if len(flange_candidates) != 1:
        raise RuntimeError(f"Expected one FLANGE.STEP solid, found {len(flange_candidates)}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    fingertip = find_fingertip_shape()
    if not all(close(value, expected) for value, expected in zip(dimensions(fingertip), (21.0, 34.0, 12.0))):
        raise RuntimeError(f"Unexpected FINGER_FINGERTIP bounds: {dimensions(fingertip)}")

    for name, shape in (("A", main_shell), ("FLANGE", flange_candidates[0]), ("FINGER_FINGERTIP", fingertip)):
        target = OUTPUT_DIR / f"{name}.STL"
        cq.exporters.export(shape, str(target), exportType="STL", tolerance=0.15, angularTolerance=0.12)
        print(f"{name}: {dimensions(shape)} mm -> {target} ({target.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
