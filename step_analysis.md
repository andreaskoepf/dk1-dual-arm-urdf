# DK1 Dual-Arm Mounting STEP File Analysis

## Source File
- **File**: `dk1_dual_arm_mounting.step` (79MB, 1.5M lines)
- **Name in CAD**: `DK1-X-Tabletop.step`
- **Format**: STEP AP214 (AUTOMOTIVE_DESIGN)
- **Generated**: 2025-12-13 by Autodesk via STEP Tools ST-DEVELOPER v20.1
- **Entities**: 408 products, 1394 assembly relationships, 48474 axis placements

## Assembly Overview

The root assembly **"DK1-X Bottom up"** is a tabletop-mounted bimanual robot system consisting of:

1. **Aluminum extrusion frame** (L-shaped structure)
   - Front crossbar: HFS8-4080-700 at Y=20mm, Z=0 (runs along X, -350 to +350mm)
   - Side rail: HFS8-4080-400 at Y=20mm, from Z=40 to Z=440 (connects front to back)
   - Back crossbar: HFS8-4080-500 at Y=20mm, Z=480 (runs along X, -250 to +250mm)
   - Camera mast: HFS6-3030-400 vertical at X=0, Z=0 (rises from front bar)
   - T-Connectors joining the rails at corners

2. **Two follower arms** (TRLC-DK1-Follower v0.2.0) on the front crossbar
3. **Two leader arms** (TRLC-DK1-Leader v0.1.0) on the back crossbar
4. **Overhead USB camera** on vertical mast at front

## STEP Coordinate System

The STEP file uses millimeters with the following axes:
- **X**: Lateral axis (left-right when facing the robot). Range: -353 to +374 mm
- **Y**: Vertical axis (up). Range: -142 to +479 mm
- **Z**: Depth axis (front-back). Z=0 is front crossbar, Z=480 is back crossbar

## Follower Arm Mounting (for URDF)

Both follower arms are mounted on the front crossbar (Z=0) at the same height (Y=40mm), symmetrically about X=0:

| Arm | STEP Position (mm) | STEP Z-axis | STEP X-axis |
|-----|-------------------|-------------|-------------|
| **Right** (v23:1) | **(290, 40, 0)** | (0, 0, -1) | (-1, 0, 0) |
| **Left** (v23:2) | **(-290, 40, 0)** | (0, 0, -1) | (-1, 0, 0) |

**Key observations:**
- Separation between arms: **580mm** (center-to-center)
- Both arms have **identical orientation** (not mirrored)
- Arm CAD Z-axis points in STEP -Z direction (toward operator / workspace)
- Arm CAD X-axis points in STEP -X direction
- Arm CAD Y-axis (computed: Z×X) = (0, 1, 0) = STEP +Y (upward)

### Arm Internal Frame

Within each follower, `link0-1` is at local position (0, 27.375, 0) mm with Z-axis along local Z. This confirms the arm's CAD Y direction corresponds to the arm's vertical axis in its URDF (base_link Z).

## URDF World Frame Convention

The dual-arm URDF uses a **Z-up** world frame:

| URDF Axis | Direction | STEP Mapping |
|-----------|-----------|--------------|
| **X** | Lateral (right = positive) | = STEP X |
| **Y** | Forward, into workspace (away from frame) | = -STEP Z |
| **Z** | Up | = STEP Y |

### Arm Mount Transforms (URDF)

The rotation from arm-local frame to URDF world is:

```
R = T @ R_step = [[-1, 0, 0],
                   [0, 0, 1],
                   [0, 1, 0]]
```

This corresponds to **RPY = (π/2, 0, π)** = (1.5707963, 0, 3.1415927).

**Physical meaning**: The arm's base_link Z-axis (joint1 rotation axis) points in world +Y direction (forward toward workspace). The arm extends forward from the mounting bar.

| Arm | URDF xyz (meters) | URDF rpy (radians) |
|-----|-------------------|--------------------|
| **Right** | (0.290, 0.0, 0.040) | (π/2, 0, π) |
| **Left** | (-0.290, 0.0, 0.040) | (π/2, 0, π) |

## Leader Arms (for reference, not in dual-arm URDF)

| Arm | STEP Position (mm) | Separation |
|-----|-------------------|------------|
| Right Leader | (210, 40, 480) | 420mm between leaders |
| Left Leader | (-210, 40, 480) | Same orientation as followers |

Leaders are on the back crossbar, 480mm behind the followers.

## Overhead Camera

| Property | STEP Value | URDF Value |
|----------|-----------|------------|
| Position | (0, 478.81, -10.65) mm | (0.0, 0.01065, 0.47881) m |
| Z-axis (optical) | (0, -0.707, -0.707) | Points forward-down at 45° |
| X-axis | (1, 0, 0) | Lateral |
| RPY | — | (-2.356, 0, 0) = (-135°, 0, 0) |

The camera is mounted ~459mm above the front bar on a 30x30 vertical mast, tilted 45° downward to view the workspace.

## Wrist Cameras

Each follower gripper assembly contains a wrist camera:
- Position relative to gripper: (0, 89.5, -20.0) mm
- Tilted ~15° (Z-axis: 0, -0.259, 0.966)
- Already defined in the single-arm URDF as `camera` link

## Zero-Pose FK Verification (Z-up frame)

At all joints = 0, the arm kinematic chain (right arm) traces:

| Point | World Position (m) |
|-------|--------------------|
| base_link | (0.290, 0.000, 0.040) |
| joint1 | (0.290, 0.062, 0.040) |
| joint2 | (0.270, 0.100, 0.040) |
| joint3 | (0.534, 0.100, 0.040) |
| joint4 | (0.290, 0.160, 0.040) |
| joint5 | (0.228, 0.203, 0.040) |
| joint6 | (0.196, 0.160, 0.040) |
| tool0 | (0.038, 0.160, 0.040) |

The arms do not collide at zero pose (right tool0 at X=0.038, left tool0 at X=-0.542).

## Assumptions and Uncertainties

1. **CAD frame = URDF base_link frame**: We assume the arm's CAD coordinate origin and orientation match the URDF base_link frame. This is standard practice when the URDF is generated from the same CAD model.

2. **Both arms same orientation**: The STEP file shows identical (not mirrored) orientations for both followers. This is confirmed by both having the same Z-axis and X-axis directions. Both arms are functionally identical, just translated laterally.

3. **No mesh for mounting structure**: The `world` link has no visual/collision geometry. The aluminum extrusion frame, T-connectors, and camera mast are not represented as meshes in the URDF.

4. **Mesh reuse**: Both arms use the same mesh files (from `trlc-dk1/urdf/follower/meshes/`). This is correct since the arms are identical (not mirrored) in the CAD.

## Overall Bounding Box

From all assembly transform positions:
- **X**: -353 to +374 mm (727mm span, lateral)
- **Y**: -142 to +479 mm (620mm span, vertical)
- **Z**: -257 to +500 mm (757mm span, depth)
