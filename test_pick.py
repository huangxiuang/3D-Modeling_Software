"""Test picking pipeline in isolation — no Qt/GUI dependency for the VTK part."""
import numpy as np
import vtk

# ── Create a minimal scene ──────────────────────────────────────
ren = vtk.vtkRenderer()
ren_win = vtk.vtkRenderWindow()
ren_win.AddRenderer(ren)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(ren_win)

# Simple sphere
sphere = vtk.vtkSphereSource()
sphere.SetCenter(0, 0, 0)
sphere.SetRadius(5)
sphere.Update()

mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection(sphere.GetOutputPort())
actor = vtk.vtkActor()
actor.SetMapper(mapper)
ren.AddActor(actor)

# Camera looking at origin
cam = ren.GetActiveCamera()
cam.SetPosition(20, 0, 0)
cam.SetFocalPoint(0, 0, 0)
cam.SetViewUp(0, 0, 1)
ren.ResetCamera()
ren_win.Render()

# Get the render window size
w, h = ren_win.GetSize()
print(f"Render window size: {w}x{h}")

# Compute center of window in VTK display coords
# VTK display origin = bottom-left
center_x = w // 2
center_y = h // 2
print(f"Picking at display coords: ({center_x}, {center_y})")

# ── Test 1: HardwarePicker ─────────────────────────────────────
hp = vtk.vtkHardwarePicker()
result = hp.Pick(center_x, center_y, 0, ren)
print(f"\n[HardwarePicker] Pick result: {result}")
if result:
    pos = np.array(hp.GetPickPosition())
    actor_ = hp.GetActor()
    print(f"  Position: {pos}")
    print(f"  Actor: {actor_}")

# ── Test 2: CellPicker ─────────────────────────────────────────
cp = vtk.vtkCellPicker()
cp.SetTolerance(0.05)
result2 = cp.Pick(center_x, center_y, 0, ren)
print(f"\n[CellPicker] Pick result: {result2}, cell_id: {cp.GetCellId()}")
if result2:
    pos2 = np.array(cp.GetPickPosition())
    actor2 = cp.GetActor()
    print(f"  Position: {pos2}")
    print(f"  Actor: {actor2}")

# ── Test 3: WorldPointPicker ───────────────────────────────────
wp = vtk.vtkWorldPointPicker()
wp.Pick(center_x, center_y, 0, ren)
pos3 = np.array(wp.GetPickPosition())
print(f"\n[WorldPointPicker] Position: {pos3}")

print("\n✅ All pickers executed without error")
