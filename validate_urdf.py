#!/usr/bin/env python3
"""
Validate the DK1 dual-arm URDF by computing forward kinematics at zero pose.

No external dependencies required (pure Python + standard library).
Optionally uses urdfpy/yourdfpy if available for full URDF parsing validation.
"""

import math
import xml.etree.ElementTree as ET
import sys
import os


def mat_mul(A, B):
    """Multiply two 3x3 matrices."""
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(M, v):
    """Multiply 3x3 matrix by 3-vector."""
    return [sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]


def rpy_to_rot(r, p, y):
    """Convert roll-pitch-yaw to 3x3 rotation matrix (R = Rz*Ry*Rx)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def identity():
    """Return 3x3 identity matrix."""
    return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def parse_xyz(s):
    """Parse 'x y z' string to list of floats."""
    return [float(x) for x in s.split()]


def parse_rpy(s):
    """Parse 'r p y' string to list of floats."""
    return [float(x) for x in s.split()]


class URDFValidator:
    def __init__(self, urdf_path):
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        self.robot_name = self.root.get("name")
        self.links = {}
        self.joints = {}
        self._parse()

    def _parse(self):
        for link in self.root.findall("link"):
            self.links[link.get("name")] = link

        for joint in self.root.findall("joint"):
            name = joint.get("name")
            origin = joint.find("origin")
            xyz = parse_xyz(origin.get("xyz", "0 0 0")) if origin is not None else [0, 0, 0]
            rpy = parse_rpy(origin.get("rpy", "0 0 0")) if origin is not None else [0, 0, 0]
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            jtype = joint.get("type")

            self.joints[name] = {
                "type": jtype,
                "parent": parent,
                "child": child,
                "xyz": xyz,
                "rpy": rpy,
                "axis": parse_xyz(joint.find("axis").get("xyz")) if joint.find("axis") is not None else [0, 0, 1],
            }

    def get_chain(self, start_link, end_link):
        """Get joint chain from start_link to end_link."""
        # Build parent map
        child_to_joint = {}
        for jname, jdata in self.joints.items():
            child_to_joint[jdata["child"]] = jname

        chain = []
        current = end_link
        while current != start_link:
            if current not in child_to_joint:
                raise ValueError(f"Cannot find path from {start_link} to {end_link}")
            jname = child_to_joint[current]
            chain.append(jname)
            current = self.joints[jname]["parent"]
        chain.reverse()
        return chain

    def fk(self, joint_chain, joint_values=None):
        """Compute FK through a chain of joints. Returns (position, rotation)."""
        pos = [0.0, 0.0, 0.0]
        rot = identity()

        for jname in joint_chain:
            jdata = self.joints[jname]
            xyz = jdata["xyz"]
            rpy = jdata["rpy"]

            # Apply this joint's fixed transform
            R_joint = rpy_to_rot(*rpy)

            # Position: pos = pos + rot * xyz
            offset = mat_vec(rot, xyz)
            pos = [pos[i] + offset[i] for i in range(3)]

            # Rotation: rot = rot * R_joint
            rot = mat_mul(rot, R_joint)

        return pos, rot

    def validate(self):
        print(f"Robot: {self.robot_name}")
        print(f"Links: {len(self.links)}")
        print(f"Joints: {len(self.joints)}")
        print()

        # Check tree structure (no cycles, single root)
        children = set()
        parents = set()
        for jdata in self.joints.values():
            children.add(jdata["child"])
            parents.add(jdata["parent"])

        roots = parents - children
        leaves = children - parents
        print(f"Root links: {roots}")
        print(f"Leaf links: {sorted(leaves)}")
        print()

        # Check all links are connected
        all_joint_links = parents | children
        orphan_links = set(self.links.keys()) - all_joint_links
        if orphan_links:
            print(f"WARNING: Orphan links (not connected): {orphan_links}")

        # Compute FK for each arm's tool0
        for side in ["right", "left"]:
            tool_link = f"{side}_tool0"
            if tool_link not in self.links:
                print(f"WARNING: {tool_link} not found")
                continue

            chain = self.get_chain("world", tool_link)
            pos, rot = self.fk(chain)
            print(f"{side.upper()} ARM (zero pose):")
            print(f"  Chain: {' -> '.join(chain)}")
            print(f"  tool0 position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
            print(f"  tool0 rotation:")
            for row in rot:
                print(f"    [{row[0]:8.4f} {row[1]:8.4f} {row[2]:8.4f}]")
            print()

        # Check arms don't intersect at zero pose
        right_chain = self.get_chain("world", "right_tool0")
        left_chain = self.get_chain("world", "left_tool0")
        right_pos, _ = self.fk(right_chain)
        left_pos, _ = self.fk(left_chain)

        dist = math.sqrt(sum((right_pos[i] - left_pos[i]) ** 2 for i in range(3)))
        print(f"Distance between tool0 endpoints: {dist:.4f} m ({dist * 1000:.1f} mm)")
        if dist < 0.05:
            print("WARNING: Arms may be intersecting at zero pose!")
        else:
            print("OK: Arms are well separated at zero pose.")
        print()

        # Check overhead camera
        if "overhead_camera" in self.links:
            cam_chain = self.get_chain("world", "overhead_camera")
            cam_pos, cam_rot = self.fk(cam_chain)
            print(f"OVERHEAD CAMERA:")
            print(f"  Position: ({cam_pos[0]:.4f}, {cam_pos[1]:.4f}, {cam_pos[2]:.4f}) m")
            # Camera optical axis is the Z column of rotation
            optical = [cam_rot[i][2] for i in range(3)]
            print(f"  Optical axis (Z): ({optical[0]:.4f}, {optical[1]:.4f}, {optical[2]:.4f})")
            print()

        # Verify mesh files exist
        print("MESH FILE CHECK:")
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        missing = []
        checked = set()
        for link in self.root.findall("link"):
            for mesh in link.findall(".//mesh"):
                fn = mesh.get("filename")
                if fn and fn not in checked:
                    checked.add(fn)
                    full_path = os.path.join(urdf_dir, fn)
                    exists = os.path.exists(full_path)
                    if not exists:
                        missing.append(fn)
        if missing:
            print(f"  MISSING ({len(missing)}):")
            for m in missing:
                print(f"    {m}")
        else:
            print(f"  All {len(checked)} unique mesh files found.")
        print()

        # Joint summary
        print("JOINT SUMMARY:")
        for side in ["right", "left"]:
            print(f"  {side.upper()}:")
            for jname, jdata in sorted(self.joints.items()):
                if jname.startswith(side) and jdata["type"] != "fixed":
                    limit = ""
                    joint_el = None
                    for j in self.root.findall("joint"):
                        if j.get("name") == jname:
                            joint_el = j
                            break
                    if joint_el is not None:
                        lim = joint_el.find("limit")
                        if lim is not None:
                            lo = float(lim.get("lower", 0))
                            hi = float(lim.get("upper", 0))
                            limit = f"  [{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]"
                    print(f"    {jname} ({jdata['type']}){limit}")

        return True


def try_urdfpy(urdf_path):
    """Try to validate with urdfpy or yourdfpy if available."""
    try:
        import urdfpy

        robot = urdfpy.URDF.load(urdf_path)
        print(f"\nurdfpy validation: OK ({robot.name}, {len(robot.links)} links)")
        return True
    except ImportError:
        pass

    try:
        import yourdfpy

        robot = yourdfpy.URDF.load(urdf_path)
        print(f"\nyourdfpy validation: OK ({len(robot.robot.links)} links)")
        return True
    except ImportError:
        pass

    print("\nNote: Neither urdfpy nor yourdfpy available for additional validation.")
    print("Install with: pip install urdfpy  OR  pip install yourdfpy")
    return False


if __name__ == "__main__":
    urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dk1_dual_arm.urdf")
    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]

    print(f"Validating: {urdf_path}\n{'=' * 60}\n")

    validator = URDFValidator(urdf_path)
    validator.validate()

    try_urdfpy(urdf_path)
