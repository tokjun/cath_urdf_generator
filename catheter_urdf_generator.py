#!/usr/bin/env python3
"""
Xacro Generator for Flexible Catheter

This script generates Xacro data for a flexible catheter modeled as a serial link
with N links and N-1 universal joints. The catheter consists of:
- Tip link (length L1)
- Bending section (N-2 links, total length L2)
- Base link (length L3)

Parameters:
- N: Number of links
- D: Outer diameter of catheter
- L1: Length of tip link
- L2: Total length of bending section
- L3: Length of base link
- K: Spring constant for rotary springs
- M: Total mass of catheter
"""

import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math
import argparse


class CatheterXacroGenerator:
    def __init__(self, N, D, L1, L2, L3, K, M):
        self.N = N
        self.D = D
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.K = K
        self.M = M
        
        self.validate_parameters()
        self.calculate_derived_values()
    
    def validate_parameters(self):
        """Validate input parameters"""
        if self.N < 3:
            raise ValueError("N must be at least 3 (tip, at least one bending link, base)")
        if any(val <= 0 for val in [self.D, self.L1, self.L2, self.L3, self.K, self.M]):
            raise ValueError("All parameters must be positive")
    
    def calculate_derived_values(self):
        """Calculate derived values from input parameters"""
        self.bending_links = self.N - 2
        self.bending_link_length = self.L2 / self.bending_links if self.bending_links > 0 else 0
        self.radius = self.D / 2
        
        # Mass distribution
        self.tip_mass = self.M * (self.L1 / (self.L1 + self.L2 + self.L3))
        self.base_mass = self.M * (self.L3 / (self.L1 + self.L2 + self.L3))
        self.bending_mass_per_link = self.M * (self.L2 / (self.L1 + self.L2 + self.L3)) / self.bending_links if self.bending_links > 0 else 0
    
    def create_link_inertia(self, mass, length, radius):
        """Calculate inertia tensor for cylindrical link"""
        ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
        izz = 0.5 * mass * radius**2
        return {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': 0, 'ixz': 0, 'iyz': 0}
    
    def add_link(self, root, name, mass, length, radius, xyz_origin="0 0 0"):
        """Add a link element to the URDF"""
        link = ET.SubElement(root, 'link', name=name)
        
        # Visual
        visual = ET.SubElement(link, 'visual')
        visual_origin = ET.SubElement(visual, 'origin', xyz=xyz_origin, rpy="0 0 0")
        visual_geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(visual_geometry, 'cylinder', radius=str(radius), length=str(length))
        visual_material = ET.SubElement(visual, 'material', name="catheter_material")
        ET.SubElement(visual_material, 'color', rgba="0.8 0.8 0.8 1.0")
        
        # Collision
        collision = ET.SubElement(link, 'collision')
        collision_origin = ET.SubElement(collision, 'origin', xyz=xyz_origin, rpy="0 0 0")
        collision_geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(collision_geometry, 'cylinder', radius=str(radius), length=str(length))
        
        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=xyz_origin, rpy="0 0 0")
        ET.SubElement(inertial, 'mass', value=str(mass))
        inertia_values = self.create_link_inertia(mass, length, radius)
        ET.SubElement(inertial, 'inertia', **{k: str(v) for k, v in inertia_values.items()})
    
    def add_joint(self, root, name, parent, child, xyz_origin, joint_type="revolute", axis="1 0 0"):
        """Add a joint element to the URDF"""
        joint = ET.SubElement(root, 'joint', name=name, type=joint_type)
        ET.SubElement(joint, 'parent', link=parent)
        ET.SubElement(joint, 'child', link=child)
        ET.SubElement(joint, 'origin', xyz=xyz_origin, rpy="0 0 0")
        ET.SubElement(joint, 'axis', xyz=axis)
        
        # Add limits and dynamics for revolute joints
        if joint_type == "revolute":
            ET.SubElement(joint, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2), 
                         effort="100", velocity="10")
            ET.SubElement(joint, 'dynamics', damping="0.1", friction="0.01")
    
    def add_spring_plugin(self, root, joint_name, spring_constant):
        """Add Gazebo plugin for spring behavior"""
        gazebo = ET.SubElement(root, 'gazebo')
        plugin = ET.SubElement(gazebo, 'plugin', name=f"{joint_name}_spring", 
                              filename="libgazebo_ros_joint_spring.so")
        ET.SubElement(plugin, 'joint_name').text = joint_name
        ET.SubElement(plugin, 'spring_constant').text = str(spring_constant)
        ET.SubElement(plugin, 'reference_position').text = "0.0"
    
    def generate_xacro(self):
        """Generate the complete Xacro"""
        robot = ET.Element('robot', name="flexible_catheter")
        robot.set('xmlns:xacro', 'http://www.ros.org/wiki/xacro')
        
        # Add xacro properties
        self.add_xacro_properties(robot)
        
        # Add macros
        self.add_catheter_link_macro(robot)
        self.add_universal_joint_macro(robot)
        
        # Use macros to create links
        ET.SubElement(robot, 'xacro:catheter_link',
                     name='tip_link', mass='${tip_mass}', length='${tip_length}',
                     radius='${catheter_radius}', xyz_origin=f'0 0 {self.L1/2}')
        
        for i in range(self.bending_links):
            ET.SubElement(robot, 'xacro:catheter_link',
                         name=f'bending_link_{i+1}', mass='${bending_mass_per_link}',
                         length='${bending_link_length}', radius='${catheter_radius}',
                         xyz_origin=f'0 0 {self.bending_link_length/2}')
        
        ET.SubElement(robot, 'xacro:catheter_link',
                     name='base_link', mass='${base_mass}', length='${base_length}',
                     radius='${catheter_radius}', xyz_origin=f'0 0 {self.L3/2}')
        
        # Use macros to create joints
        current_z = self.L1
        
        if self.bending_links > 0:
            ET.SubElement(robot, 'xacro:universal_joint',
                         name='tip_to_bending_1', parent='tip_link', child='bending_link_1',
                         xyz_origin=f'0 0 {current_z}', spring_k='${spring_constant}')
            current_z += self.bending_link_length
            
            for i in range(1, self.bending_links):
                ET.SubElement(robot, 'xacro:universal_joint',
                             name=f'bending_{i}_to_{i+1}', parent=f'bending_link_{i}',
                             child=f'bending_link_{i+1}', xyz_origin=f'0 0 {current_z}',
                             spring_k='${spring_constant}')
                current_z += self.bending_link_length
            
            ET.SubElement(robot, 'xacro:universal_joint',
                         name=f'bending_{self.bending_links}_to_base',
                         parent=f'bending_link_{self.bending_links}', child='base_link',
                         xyz_origin=f'0 0 {current_z}', spring_k='${spring_constant}')
        else:
            ET.SubElement(robot, 'xacro:universal_joint',
                         name='tip_to_base', parent='tip_link', child='base_link',
                         xyz_origin=f'0 0 {current_z}', spring_k='${spring_constant}')
        
        return robot
    
    def add_xacro_properties(self, root):
        """Add xacro properties for parameters"""
        ET.SubElement(root, 'xacro:property', name='catheter_links', value=str(self.N))
        ET.SubElement(root, 'xacro:property', name='catheter_diameter', value=str(self.D))
        ET.SubElement(root, 'xacro:property', name='catheter_radius', value=str(self.radius))
        ET.SubElement(root, 'xacro:property', name='tip_length', value=str(self.L1))
        ET.SubElement(root, 'xacro:property', name='bending_length', value=str(self.L2))
        ET.SubElement(root, 'xacro:property', name='base_length', value=str(self.L3))
        ET.SubElement(root, 'xacro:property', name='spring_constant', value=str(self.K))
        ET.SubElement(root, 'xacro:property', name='total_mass', value=str(self.M))
        ET.SubElement(root, 'xacro:property', name='bending_links', value=str(self.bending_links))
        ET.SubElement(root, 'xacro:property', name='bending_link_length', value=str(self.bending_link_length))
        ET.SubElement(root, 'xacro:property', name='tip_mass', value=str(self.tip_mass))
        ET.SubElement(root, 'xacro:property', name='base_mass', value=str(self.base_mass))
        ET.SubElement(root, 'xacro:property', name='bending_mass_per_link', value=str(self.bending_mass_per_link))
        
        # Add material definition
        material = ET.SubElement(root, 'material', name='catheter_material')
        ET.SubElement(material, 'color', rgba='0.8 0.8 0.8 1.0')
    
    def add_catheter_link_macro(self, root):
        """Add xacro macro for catheter links"""
        macro = ET.SubElement(root, 'xacro:macro', name='catheter_link')
        macro.set('params', 'name mass length radius xyz_origin')
        
        link = ET.SubElement(macro, 'link', name='${name}')
        
        # Visual
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        visual_geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(visual_geometry, 'cylinder', radius='${radius}', length='${length}')
        ET.SubElement(visual, 'material', name='catheter_material')
        
        # Collision
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        collision_geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(collision_geometry, 'cylinder', radius='${radius}', length='${length}')
        
        # Inertial
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value='${mass}')
        
        # Calculate inertia using xacro expressions
        ixx_iyy = '${(1/12) * mass * (3 * radius * radius + length * length)}'
        izz = '${0.5 * mass * radius * radius}'
        
        ET.SubElement(inertial, 'inertia', 
                     ixx=ixx_iyy, iyy=ixx_iyy, izz=izz,
                     ixy='0', ixz='0', iyz='0')
    
    def add_universal_joint_macro(self, root):
        """Add xacro macro for universal joints with springs"""
        macro = ET.SubElement(root, 'xacro:macro', name='universal_joint')
        macro.set('params', 'name parent child xyz_origin spring_k')
        
        # X-axis joint
        joint_x = ET.SubElement(macro, 'joint', name='${name}_x', type='revolute')
        ET.SubElement(joint_x, 'parent', link='${parent}')
        ET.SubElement(joint_x, 'child', link='${name}_x_rotation')
        ET.SubElement(joint_x, 'origin', xyz='${xyz_origin}', rpy='0 0 0')
        ET.SubElement(joint_x, 'axis', xyz='1 0 0')
        ET.SubElement(joint_x, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2), 
                     effort='100', velocity='10')
        ET.SubElement(joint_x, 'dynamics', damping='0.1', friction='0.01')
        
        # Intermediate link for universal joint
        intermediate_link = ET.SubElement(macro, 'link', name='${name}_x_rotation')
        inertial = ET.SubElement(intermediate_link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.001')
        ET.SubElement(inertial, 'inertia', ixx='1e-6', iyy='1e-6', izz='1e-6',
                     ixy='0', ixz='0', iyz='0')
        
        # Y-axis joint
        joint_y = ET.SubElement(macro, 'joint', name='${name}_y', type='revolute')
        ET.SubElement(joint_y, 'parent', link='${name}_x_rotation')
        ET.SubElement(joint_y, 'child', link='${child}')
        ET.SubElement(joint_y, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(joint_y, 'axis', xyz='0 1 0')
        ET.SubElement(joint_y, 'limit', lower=str(-math.pi/2), upper=str(math.pi/2), 
                     effort='100', velocity='10')
        ET.SubElement(joint_y, 'dynamics', damping='0.1', friction='0.01')
        
        # Spring plugins
        gazebo_x = ET.SubElement(macro, 'gazebo')
        plugin_x = ET.SubElement(gazebo_x, 'plugin', name='${name}_x_spring',
                                filename='libgazebo_ros_joint_spring.so')
        joint_name_x = ET.SubElement(plugin_x, 'joint_name')
        joint_name_x.text = '${name}_x'
        spring_const_x = ET.SubElement(plugin_x, 'spring_constant')
        spring_const_x.text = '${spring_k}'
        ref_pos_x = ET.SubElement(plugin_x, 'reference_position')
        ref_pos_x.text = '0.0'
        
        gazebo_y = ET.SubElement(macro, 'gazebo')
        plugin_y = ET.SubElement(gazebo_y, 'plugin', name='${name}_y_spring',
                                filename='libgazebo_ros_joint_spring.so')
        joint_name_y = ET.SubElement(plugin_y, 'joint_name')
        joint_name_y.text = '${name}_y'
        spring_const_y = ET.SubElement(plugin_y, 'spring_constant')
        spring_const_y.text = '${spring_k}'
        ref_pos_y = ET.SubElement(plugin_y, 'reference_position')
        ref_pos_y.text = '0.0'
    
    def save_xacro(self, filename="flexible_catheter.xacro"):
        """Save the Xacro to file"""
        robot = self.generate_xacro()
        
        # Pretty print XML
        rough_string = ET.tostring(robot, 'unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        # Remove empty lines
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        final_xml = '\n'.join(lines)
        
        with open(filename, 'w') as f:
            f.write(final_xml)
        
        print(f"Xacro saved to {filename}")


def main():
    parser = argparse.ArgumentParser(description="Generate Xacro for flexible catheter")
    parser.add_argument("--N", type=int, default=5, help="Number of links")
    parser.add_argument("--D", type=float, default=0.002, help="Outer diameter (m)")
    parser.add_argument("--L1", type=float, default=0.01, help="Tip link length (m)")
    parser.add_argument("--L2", type=float, default=0.1, help="Bending section length (m)")
    parser.add_argument("--L3", type=float, default=0.05, help="Base link length (m)")
    parser.add_argument("--K", type=float, default=0.1, help="Spring constant (Nm/rad)")
    parser.add_argument("--M", type=float, default=0.01, help="Total mass (kg)")
    parser.add_argument("--output", type=str, default="flexible_catheter.xacro", 
                       help="Output filename")
    
    args = parser.parse_args()
    
    try:
        generator = CatheterXacroGenerator(args.N, args.D, args.L1, args.L2, args.L3, args.K, args.M)
        generator.save_xacro(args.output)
        
        print(f"Generated Xacro with parameters:")
        print(f"  Links: {args.N}")
        print(f"  Diameter: {args.D} m")
        print(f"  Tip length: {args.L1} m")
        print(f"  Bending section length: {args.L2} m")
        print(f"  Base length: {args.L3} m")
        print(f"  Spring constant: {args.K} Nm/rad")
        print(f"  Total mass: {args.M} kg")
        
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())