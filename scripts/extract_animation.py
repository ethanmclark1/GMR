import json
import mujoco
import pickle

def extract_animation_to_file(pkl_file, robot_xml, output_json):
    """Extract animation with joint axis information"""
    model = mujoco.MjModel.from_xml_path(robot_xml)
    data = mujoco.MjData(model)
    
    with open(pkl_file, 'rb') as f:
        motion_data = pickle.load(f)
    
    # Get bind pose
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    
    # Extract skeleton
    skeleton_info = {
        'body_names': [],
        'parent_indices': [],
        'bind_pose_positions': [],
        'bind_pose_rotations': [],
        'body_to_joint_map': {}
    }
    
    for i in range(model.nbody):
        body = model.body(i)
        skeleton_info['body_names'].append(body.name)
        skeleton_info['parent_indices'].append(int(model.body_parentid[i]))
        skeleton_info['bind_pose_positions'].append(data.xpos[i].tolist())
        skeleton_info['bind_pose_rotations'].append(data.xquat[i].tolist())
    
    # Map bodies to joints WITH AXIS INFO
    print("\nBody to Joint mapping with axes:")
    for i in range(model.njnt):
        joint = model.joint(i)
        body_id = model.jnt_bodyid[i]
        body_name = model.body(body_id).name
        qpos_adr = model.jnt_qposadr[i]
        
        # Get joint axis
        axis = model.jnt_axis[i].tolist()
        
        skeleton_info['body_to_joint_map'][body_name] = {
            'joint_name': joint.name,
            'qpos_address': int(qpos_adr),
            'body_index': int(body_id),
            'axis': axis  # [x, y, z]
        }
        print(f"  {body_name:30s} axis: {axis}")
    
    # Extract animation
    animation_frames = []
    num_frames = len(motion_data['root_pos'])
    
    print(f"\nProcessing {num_frames} frames...")
    for i in range(num_frames):
        if i % 500 == 0:
            print(f"  Frame {i}/{num_frames}")
        
        frame_data = {
            'frame_index': i,
            'root_position': motion_data['root_pos'][i].tolist(),
            'root_rotation': motion_data['root_rot'][i].tolist(),
            'joint_angles': motion_data['dof_pos'][i].tolist()
        }
        animation_frames.append(frame_data)
    
    output_data = {
        'metadata': {
            'num_frames': num_frames,
            'fps': motion_data.get('fps', 30),
        },
        'skeleton': skeleton_info,
        'animation': animation_frames
    }
    
    with open(output_json, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"\n✓ Saved to {output_json}")

if __name__ == "__main__":
    extract_animation_to_file(
        pkl_file='motion_data/walk1_subject1.pkl',
        robot_xml='assets/unitree_g1/g1_mocap_29dof.xml',
        output_json='motion_data/walk1_subject1.json'
    )