import json
import mujoco
import pickle
import numpy as np

def extract_animation_to_file(pkl_file, robot_xml, output_json):
    """Extract complete animation data to JSON"""
    model = mujoco.MjModel.from_xml_path(robot_xml)
    data = mujoco.MjData(model)
    
    with open(pkl_file, 'rb') as f:
        motion_data = pickle.load(f)
    
    # Extract bind pose (qpos = 0)
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    
    # Extract skeleton structure
    skeleton_info = {
        'body_names': [],
        'parent_indices': [],
        'local_offsets': [],
        'local_rotations': [],
        'bind_pose_global_positions': [],
        'bind_pose_global_rotations': []
    }
    
    for i in range(model.nbody):
        body = model.body(i)
        skeleton_info['body_names'].append(body.name)
        skeleton_info['parent_indices'].append(int(model.body_parentid[i]))
        skeleton_info['local_offsets'].append(model.body_pos[i].tolist())
        skeleton_info['local_rotations'].append(model.body_quat[i].tolist())
        skeleton_info['bind_pose_global_positions'].append(data.xpos[i].tolist())
        skeleton_info['bind_pose_global_rotations'].append(data.xquat[i].tolist())
    
    # Extract animation frames
    animation_frames = []
    num_frames = len(motion_data['root_pos'])
    
    print(f"Processing {num_frames} frames...")
    for i in range(num_frames):
        if i % 500 == 0:
            print(f"  Frame {i}/{num_frames}")
        
        base_pos = motion_data['root_pos'][i]
        base_rot = motion_data['root_rot'][i]
        joint_angles = motion_data['dof_pos'][i]
        
        data.qpos[:3] = base_pos
        data.qpos[3:7] = base_rot
        data.qpos[7:] = joint_angles
        
        mujoco.mj_forward(model, data)
        
        frame_data = {
            'frame_index': i,
            'positions': data.xpos.tolist(),
            'rotations': data.xquat.tolist()
        }
        animation_frames.append(frame_data)
    
    # Combine everything
    output_data = {
        'metadata': {
            'source_pkl': pkl_file,
            'source_xml': robot_xml,
            'num_frames': num_frames,
            'num_bodies': model.nbody,
            'fps': motion_data.get('fps', 30),
            'format_version': '1.0'
        },
        'skeleton': skeleton_info,
        'animation': animation_frames
    }
    
    # Save to JSON
    with open(output_json, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"\n✓ Saved to {output_json}")
    return output_data

if __name__ == "__main__":
    extract_animation_to_file(
        pkl_file='motion_data/walk1_subject1.pkl',
        robot_xml='assets/unitree_g1/g1_mocap_29dof.xml',
        output_json='motion_data/walk1_subject1.json'
    )