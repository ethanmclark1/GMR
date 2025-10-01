import mujoco
import pickle
import numpy as np
import json

def extract_animation_to_file(pkl_file, robot_xml, output_json):
    """Extract bone transforms from PKL and save to JSON"""
    model = mujoco.MjModel.from_xml_path(robot_xml)
    data = mujoco.MjData(model)
    
    with open(pkl_file, 'rb') as f:
        motion_data = pickle.load(f)
    
    animation_frames = [{
        'body_names': [model.body(i).name for i in range(model.nbody)],
    }]
    
    for i in range(len(motion_data)):
        base_pos = motion_data['root_pos'][i]
        base_rot = motion_data['root_rot'][i]
        joint_angles = motion_data['dof_pos'][i]
        
        # Set state
        data.qpos[:3] = base_pos
        data.qpos[3:7] = base_rot
        data.qpos[7:] = joint_angles
        
        # Compute forward kinematics
        mujoco.mj_forward(model, data)
        
        # Extract all body transforms
        frame_data = {
            'frame_index': i,
            'positions': data.xpos.tolist(),
            'rotations': data.xquat.tolist()
        }
        animation_frames.append(frame_data)
    
    # Save to JSON file
    with open(output_json, 'w') as f:
        json.dump(animation_frames, f, indent=2)
    
    print(f"Saved animation data to {output_json}")
    return animation_frames

if __name__ == "__main__":
    extract_animation_to_file(
        pkl_file='robot_animation.pkl',
        robot_xml='robots/unitree_g1/scene.xml',
        output_json='animation_data.json'
    )