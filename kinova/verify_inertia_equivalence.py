"""
Verify that the quaternion-based and fullinertia-based XML models are equivalent.

This script:
1. Converts quaternion + diaginertia to fullinertia representation
2. Creates a new XML with fullinertia values
3. Compares inverse dynamics (mj_inverse) results between both models
   using random joint positions and velocities
"""

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET
import re


def quat_to_rotation_matrix(quat):
    """Convert MuJoCo quaternion (w, x, y, z) to rotation matrix."""
    w, x, y, z = quat
    # Convert to scipy format (x, y, z, w)
    r = Rotation.from_quat([x, y, z, w])
    return r.as_matrix()


def diaginertia_to_fullinertia(quat, diaginertia):
    """
    Convert diagonal inertia in principal frame to full inertia in body frame.
    
    The inertia tensor in the principal frame is diagonal:
    I_principal = diag(Ixx, Iyy, Izz)
    
    The rotation matrix R transforms from principal to body frame:
    I_body = R @ I_principal @ R.T
    
    Returns fullinertia as (Ixx, Iyy, Izz, Ixy, Ixz, Iyz) in body frame.
    """
    R = quat_to_rotation_matrix(quat)
    I_principal = np.diag(diaginertia)
    I_body = R @ I_principal @ R.T
    
    # Extract components (MuJoCo fullinertia order: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
    Ixx = I_body[0, 0]
    Iyy = I_body[1, 1]
    Izz = I_body[2, 2]
    Ixy = I_body[0, 1]
    Ixz = I_body[0, 2]
    Iyz = I_body[1, 2]
    
    return np.array([Ixx, Iyy, Izz, Ixy, Ixz, Iyz])


def parse_inertial_from_xml(xml_path):
    """Parse inertial elements from XML and extract quat + diaginertia."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    inertials = []
    for inertial in root.iter('inertial'):
        data = {
            'pos': np.array([float(x) for x in inertial.get('pos').split()]),
            'mass': float(inertial.get('mass'))
        }
        
        if inertial.get('quat') is not None:
            data['quat'] = np.array([float(x) for x in inertial.get('quat').split()])
            data['diaginertia'] = np.array([float(x) for x in inertial.get('diaginertia').split()])
        elif inertial.get('fullinertia') is not None:
            data['fullinertia'] = np.array([float(x) for x in inertial.get('fullinertia').split()])
        
        inertials.append(data)
    
    return inertials


def create_fullinertia_xml(input_path, output_path):
    """Create a new XML file with fullinertia instead of quat+diaginertia."""
    tree = ET.parse(input_path)
    root = tree.getroot()
    
    conversions = []
    
    for inertial in root.iter('inertial'):
        if inertial.get('quat') is not None and inertial.get('diaginertia') is not None:
            quat = np.array([float(x) for x in inertial.get('quat').split()])
            diaginertia = np.array([float(x) for x in inertial.get('diaginertia').split()])
            
            fullinertia = diaginertia_to_fullinertia(quat, diaginertia)
            
            conversions.append({
                'quat': quat,
                'diaginertia': diaginertia,
                'fullinertia': fullinertia
            })
            
            # Remove quat and diaginertia, add fullinertia
            del inertial.attrib['quat']
            del inertial.attrib['diaginertia']
            inertial.set('fullinertia', ' '.join(f'{x:.8g}' for x in fullinertia))
    
    tree.write(output_path, encoding='unicode')
    
    # Add XML declaration and fix formatting
    with open(output_path, 'r') as f:
        content = f.read()
    
    return conversions


def verify_inverse_dynamics(xml_path1, xml_path2, num_tests=100, seed=42):
    """
    Verify that two models produce identical inverse dynamics results.
    
    Uses mj_inverse to compute required forces for random states.
    """
    np.random.seed(seed)
    
    model1 = mujoco.MjModel.from_xml_path(xml_path1)
    model2 = mujoco.MjModel.from_xml_path(xml_path2)
    
    data1 = mujoco.MjData(model1)
    data2 = mujoco.MjData(model2)
    
    nq = model1.nq
    nv = model1.nv
    
    max_force_diff = 0.0
    max_rel_diff = 0.0
    all_diffs = []
    
    print(f"\nRunning {num_tests} inverse dynamics comparisons...")
    print(f"Model 1: {xml_path1}")
    print(f"Model 2: {xml_path2}")
    print(f"DOF: {nv}")
    print("-" * 60)
    
    for i in range(num_tests):
        # Random joint positions within limits
        qpos = np.random.uniform(-2.0, 2.0, nq)
        
        # Random joint velocities
        qvel = np.random.uniform(-3.0, 3.0, nv)
        
        # Random joint accelerations
        qacc = np.random.uniform(-5.0, 5.0, nv)
        
        # Set state for model 1
        data1.qpos[:] = qpos
        data1.qvel[:] = qvel
        data1.qacc[:] = qacc
        mujoco.mj_inverse(model1, data1)
        forces1 = data1.qfrc_inverse.copy()
        
        # Set state for model 2
        data2.qpos[:] = qpos
        data2.qvel[:] = qvel
        data2.qacc[:] = qacc
        mujoco.mj_inverse(model2, data2)
        forces2 = data2.qfrc_inverse.copy()
        
        # Compare forces
        diff = np.abs(forces1 - forces2)
        max_diff = np.max(diff)
        all_diffs.append(max_diff)
        
        if max_diff > max_force_diff:
            max_force_diff = max_diff
        
        # Relative difference (avoid division by zero)
        denom = np.maximum(np.abs(forces1), np.abs(forces2))
        denom = np.where(denom < 1e-10, 1.0, denom)
        rel_diff = np.max(diff / denom)
        if rel_diff > max_rel_diff:
            max_rel_diff = rel_diff
    
    print(f"\nResults:")
    print(f"  Max absolute force difference: {max_force_diff:.2e} N·m")
    print(f"  Max relative force difference: {max_rel_diff:.2e}")
    print(f"  Mean absolute difference: {np.mean(all_diffs):.2e} N·m")
    print(f"  Std absolute difference: {np.std(all_diffs):.2e} N·m")
    
    # Check if models are equivalent (within numerical tolerance)
    # Use relative tolerance since absolute values vary with state
    rel_tolerance = 1e-6
    abs_tolerance = 1e-3  # 1 mN·m is negligible for robot control
    
    if max_rel_diff < rel_tolerance and max_force_diff < abs_tolerance:
        print(f"\n✓ Models are EQUIVALENT (rel_diff < {rel_tolerance}, abs_diff < {abs_tolerance})")
        return True
    else:
        print(f"\n✗ Models differ: rel_diff={max_rel_diff:.2e} (tol={rel_tolerance}), abs_diff={max_force_diff:.2e} (tol={abs_tolerance})")
        return False


def print_inertia_comparison(xml_path):
    """Print the inertia values from the original XML and computed fullinertia."""
    inertials = parse_inertial_from_xml(xml_path)
    
    print("\nInertia Conversion Details:")
    print("=" * 80)
    
    for i, inertial in enumerate(inertials):
        print(f"\nLink {i+1}:")
        print(f"  Position: {inertial['pos']}")
        print(f"  Mass: {inertial['mass']}")
        
        if 'quat' in inertial:
            quat = inertial['quat']
            diaginertia = inertial['diaginertia']
            fullinertia = diaginertia_to_fullinertia(quat, diaginertia)
            
            print(f"  Quaternion (w,x,y,z): {quat}")
            print(f"  Diagonal Inertia: {diaginertia}")
            print(f"  → Full Inertia (Ixx,Iyy,Izz,Ixy,Ixz,Iyz): {fullinertia}")
            
            # Verify by reconstructing
            R = quat_to_rotation_matrix(quat)
            I_body = R @ np.diag(diaginertia) @ R.T
            print(f"  → Full Inertia Matrix:\n{I_body}")


def compare_body_inertias(model1, model2):
    """Compare the body inertia matrices between two models."""
    print("\nBody Inertia Comparison (from compiled models):")
    print("=" * 80)
    
    for i in range(model1.nbody):
        name = mujoco.mj_id2name(model1, mujoco.mjtObj.mjOBJ_BODY, i)
        
        # Get inertia from both models
        inertia1 = model1.body_inertia[i]
        inertia2 = model2.body_inertia[i]
        
        diff = np.abs(inertia1 - inertia2)
        max_diff = np.max(diff)
        
        print(f"\n{name}:")
        print(f"  Model 1 inertia: {inertia1}")
        print(f"  Model 2 inertia: {inertia2}")
        print(f"  Max difference: {max_diff:.2e}")


if __name__ == "__main__":
    import os
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    original_xml = os.path.join(script_dir, "model", "kinova.xml")
    fullinertia_xml = os.path.join(script_dir, "model", "kinova_fullinertia.xml")
    
    # Print conversion details
    print_inertia_comparison(original_xml)
    
    # Load both models and compare body inertias
    print("\n" + "=" * 80)
    model1 = mujoco.MjModel.from_xml_path(original_xml)
    model2 = mujoco.MjModel.from_xml_path(fullinertia_xml)
    compare_body_inertias(model1, model2)
    
    # Verify inverse dynamics equivalence
    print("\n" + "=" * 80)
    verify_inverse_dynamics(original_xml, fullinertia_xml, num_tests=100)
