# Catheter Xacro Generator

A Python script to generate Xacro data for flexible catheters modeled as serial links with universal joints and rotary springs.

## Usage

### Basic Usage

```bash
python3 catheter_urdf_generator.py [OPTIONS]
```

### Parameters

- `--N`: Number of links (default: 5)
- `--D`: Outer diameter in meters (default: 0.002)
- `--L1`: Tip link length in meters (default: 0.01)
- `--L2`: Bending section total length in meters (default: 0.1)
- `--L3`: Base link length in meters (default: 0.05)
- `--K`: Spring constant in Nm/rad (default: 0.1)
- `--M`: Total mass in kg (default: 0.01)
- `--output`: Output filename (default: flexible_catheter.xacro)

### Examples

Generate a catheter with default parameters:
```bash
python3 catheter_urdf_generator.py
```

Generate a custom catheter:
```bash
python3 catheter_urdf_generator.py --N 7 --D 0.003 --L1 0.015 --L2 0.08 --L3 0.04 --K 0.2 --M 0.008 --output my_catheter.xacro
```

### Output

The script generates a Xacro file with:
- N links (tip, bending section, base) using parametric macros
- N-1 universal joints with rotary springs
- Xacro properties for all parameters (reusable and configurable)
- Xacro macros for repeated link and joint structures
- Proper mass distribution and inertia calculations
- Gazebo spring plugins for flexible behavior

### Converting to URDF

To convert the generated Xacro file to URDF format:
```bash
rosrun xacro xacro flexible_catheter.xacro > flexible_catheter.urdf
```

Or with ROS2:
```bash
ros2 run xacro xacro flexible_catheter.xacro > flexible_catheter.urdf
```

## Development
I would like to make a Python script to generate an URDF data of a flexible catheter. The user can provide marameters, including N, D, L1, L2, L3, K, and M. Since URDF does not support flexible object, the script generate a serial link that mimics the flexible catheter. The serial link has N links and N-1 universal joints. The first and the last links are the tip and the base links, and the reminders are in the bending seciton. Each joints have rotary springs with spring constant of K that generate torques to bring it back to the straight position, when the joint is rotated by the external force. D is the outer diameter of the catheter. L1, L2, and L3 are the lengths of the tip link, the bending section, and the base link. The total weight of the catheter is M.

