# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ROS1 (Noetic) catkin workspace for the **Teknofest FDA** ("Fırlatılabilir Döner Kanat") competition: a Gazebo simulation of the course (UAV launched from a pneumatic catapult, flies through the HTAB obstacle field, lands on a target helipad) plus the beginnings of a downward-camera perception stack.

Two halves that are currently **not wired together**:

1. **The arena** — `worlds/arena.world` (world `htab_world`) opened by `launch/sim.launch`. Contains the ground, the helipad, ~53 rubble piles, 10 jersey barriers, and a static downward-looking camera. No UAV model/URDF exists yet, so nothing is spawned and nothing flies.
2. **Perception** — `scripts/` (`camera_listener.py`, `detect_target.py`, `costmap.py`) + `best.pt` (YOLO weights). Run manually against the camera topic; not started by any launch file.

Comments and docstrings are in **Turkish** and frequently cite the competition rulebook ("şartname") by section (e.g. "şartname 2.1"). Keep new comments in the same language and style, and preserve the derivation/rationale blocks — several of them are the only record of how a number was computed.

## Commands

From `ros1_ws/`:

```bash
catkin_make                          # build (nothing but catkin_package() is enabled; this mostly just sets up devel/)
source devel/setup.bash              # required before roslaunch/rosrun
roslaunch fda-teknofest sim.launch   # open worlds/arena.world in Gazebo
```

Perception node (separate terminal, after the sim is up):

```bash
rosrun fda-teknofest camera_listener.py    # subscribes /camera/image_raw, publishes camera/yolo_frame
```

`rosrun` works because `camera_listener.py` is chmod +x and found in the source tree — `catkin_install_python()` in `CMakeLists.txt` is still commented out, so nothing is installed into `devel/lib/`. Mark any new node executable the same way, or enable the install rule.

`ultralytics`, `opencv-python` and `numpy` are pip dependencies and are **not** declared in `package.xml` (which lists only `gazebo_msgs`, `gazebo_ros`, `rospy`, `sensor_msgs`, `cv_bridge`).

No tests, linter, or CI exist; the testing section of `CMakeLists.txt` is fully commented out.

## `worlds/arena.world` — read this before editing anything geometric

The world was **saved from the Gazebo GUI**, which changes how it must be edited:

- **Models are inlined, not `model://` includes.** Every rubble pile and barrier is a full copy pasted into the world (`rubble_pile`, `rubble_pile_clone`, `rubble_pile_clone_17_clone_24_clone_5`, …). Editing `models/<name>/model.sdf` therefore has **no effect on the live scene** — the world holds its own copies. The only surviving `model://` URIs are mesh/material paths, which is why `sim.launch` still has to set `GAZEBO_MODEL_PATH`.
- **The `<state world_name='htab_world'>` block (lines ~252–910) wins.** It carries a pose for all 65 models and Gazebo applies it on load, overriding the `<pose>` in the model definition further down. `target_helipad` is defined at `0 20 0` but actually sits at `3.55 5.62 0`. When you need a real obstacle position — for path checks, gate placement, spawn poses — read it from `<state>`, which is what `models/htab_ayirici`'s A*/gate derivation did.
- Layout order is unusual: light → `dirt_ground` → `target_helipad` → `camera` → physics/scene → `<state>` → `<gui>` → the 63 rubble/barrier definitions.
- `mancinik`, `parasut` and `htab_ayirici` are modeled under `models/` but are **not in the world**. The three-route HTAB structure the rulebook describes is therefore not yet enforced in the live scene.

## Perception pipeline (`scripts/`)

- `camera_listener.py` — the only ROS node. Loads `best.pt` (path resolved relative to the script: `scripts/../best.pt`), subscribes `/camera/image_raw`, runs `DetectTarget` for the red helipad and YOLO for segmentation, republishes the annotated frame on `camera/yolo_frame` for debugging. Known gaps flagged in its own comments: images are not undistorted before YOLO, `verbose=True` should be off in production, and the YOLO mask is fetched but unused.
- `detect_target.py` — `DetectTarget`: HSV red thresholding (two hue bands) + morphological close + largest connected component above `min_area`; returns `(h_mask, (cx, cy))`, or a black mask and `(None, None)` when nothing qualifies.
- `costmap.py` — `CostMap`: 1200×1200 grid at 0.1 m/px, UAV-centered. `calculate_H(roll, pitch, yaw, uav_x, uav_y, altitude)` builds the image→ground→costmap homography; `update(mask, H)` warps a YOLO class mask (1=road, 2=not_road, 3=target) into the grid and integrates it into `guven_haritasi` (confidence, −100..100) before thresholding into costs 0/128/255. Refuses to compute below `MIN_ALTITUDE = 10 m`. **Construct once and call `update`/`calculate_H` per frame** — re-instantiating throws away the accumulated confidence map. Not yet imported by any node.

### Cross-file invariants

- **Camera intrinsics.** The world's `camera` sensor is modeled on the real hardware (Raspberry Pi Global Shutter / IMX296 + 6 mm lens): `hfov = 0.792875`, render size `1456×1088` (deliberately native, to keep sim-to-real pixel thresholds identical), `fx = fy = 1739.1304`, `cx = 728`, `cy = 544`. Those same four numbers must be passed to `CostMap(...)`; if the two drift apart the ground projection silently shifts. The derivation is in the SDF comment block at `worlds/arena.world:143`.
- **Camera altitude.** The camera sits at `pose 0 0 10 0 1.5708 0` (nadir, 10 m), exactly at `CostMap.MIN_ALTITUDE`. Lowering it makes `calculate_H` return `None`.
- **`CostMap.R_bc`** (camera→body rotation) is marked as unverified against the actual camera mounting.
- **Catapult geometry.** `models/mancinik/model.sdf` derives rail-end and UAV rest coordinates from a 4.0 m rail at 10°; its header names the spawn pose (`x = 1.4529`, `z = 0.9984`) that a future UAV spawn in `sim.launch` is expected to match. Resizing the rail means updating every dependent pose.

## Models (`ros1_ws/src/fda-teknofest/models/`)

Standalone Gazebo models (`model.config` + `model.sdf`, sometimes `meshes/`/`materials/`), used as the source for what was pasted into the world:

- `dirt_ground` — 300×300 m textured ground plane.
- `target_helipad` — red "H" landing target (what `DetectTarget` keys on).
- `rubble_pile`, `jersey_barrier` — obstacle dressing; `jersey_barrier` (4.065 m, third-party mesh, see its `model.config`) is sized to fully block one 3.0 m HTAB gate.
- `htab_ayirici` — wall splitting HTAB into three routes at `y = 5.0`, gates at `x = -15.5 / 9.65 / 21.45` (şartname 2.1: only one route is open on competition day). Gate positions were validated with A* against the world `<state>` poses; they stay valid only as long as those poses don't move.
- `mancinik` — pneumatic catapult, **static single link**. Launch dynamics are not simulated; they are meant to be applied externally by a `scripts/mancinik.py` that **does not exist**.
- `parasut` — visual-only canopy (`kinematic`, gravity off). Its mesh URI is still the literal placeholder `__CANOPY_MESH_URI__`, so the model will not load as written; the real mesh is at `meshes/canopy.obj`. Aerodynamics are meant to live in a `scripts/parasut.py` that **does not exist**.

When adding a model, follow the existing pattern: a header comment explaining the *why* (rulebook citation, derived geometry, what physics is and isn't modeled), an explicit `<static>`, and Gazebo stock materials for simple geometry rather than new mesh assets. Remember that adding it to `models/` does not put it in the world — it has to be inlined into `arena.world` (and given a `<state>` entry) to appear.

## Layout notes

- `ros1_ws/src/CMakeLists.txt` is a symlink to catkin's `toplevel.cmake` — do not edit.
- `ros1_ws/build/`, `ros1_ws/devel/`, `scripts/__pycache__/` are generated.
- `ros1_ws/src/fda-teknofest/src/` is empty; Python nodes live in `scripts/`.
- `best.pt` (3.4 MB) sits at the package root and is referenced by relative path from `scripts/`.
- This directory is not a git repository.
