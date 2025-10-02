import json
import mujoco
import pickle


def extract_to_json(pkl_file, robot_xml, output_json):
    """Extract complete animation data for FBX export"""
    model = mujoco.MjModel.from_xml_path(robot_xml)
    data = mujoco.MjData(model)
    
    with open(pkl_file, 'rb') as f:
        motion_data = pickle.load(f)
    
    # Set the bind/default pose
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    
    # Extract skeleton with local transforms
    skeleton_info = {
        'body_names': [],
        'parent_indices': [],
        'bind_pose_local_pos': [],
        'bind_pose_local_rot': [],
        'bind_pose_world_pos': [],
       'bind_pose_world_rot': [],
        'body_to_joint_map': {}
    }
    
    for i in range(model.nbody):
        body = model.body(i)
        skeleton_info['body_names'].append(body.name)
        parent_id = int(model.body_parentid[i])
        skeleton_info['parent_indices'].append(parent_id)
        
        # World space
        skeleton_info['bind_pose_world_pos'].append(data.xpos[i].tolist())
        skeleton_info['bind_pose_world_rot'].append(data.xquat[i].tolist())
        
        # local space (from MuJoCo model definition)
        skeleton_info['bind_pose_local_pos'].append(model.body_pos[i].tolist())
        skeleton_info['bind_pose_local_rot'].append(model.body_quat[i].tolist())
    
    # Map bodies to joints with full joint info
    print("\nJoint mapping:")
    for i in range(model.njnt):
        joint = model.joint(i)
        body_id = model.jnt_bodyid[i]
        body_name = model.body(body_id).name
        qpos_adr = model.jnt_qposadr[i]
        
        joint_type_map = {
            mujoco.mjtJoint.mjJNT_FREE: 'free',
            mujoco.mjtJoint.mjJNT_BALL: 'ball',
            mujoco.mjtJoint.mjJNT_SLIDE: 'slide',
            mujoco.mjtJoint.mjJNT_HINGE: 'hinge'
        }
        
        joint_info = {
            'joint_name': joint.name,
            'qpos_address': int(qpos_adr),
            'body_index': int(body_id),
            'axis': model.jnt_axis[i].tolist(),
            'type': joint_type_map.get(model.jnt_type[i], 'unknown'),
            'range': model.jnt_range[i].tolist() if model.jnt_limited[i] else None,
            'dof_count': 3 if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_BALL else 1
        }
        
        skeleton_info['body_to_joint_map'][body_name] = joint_info
        print(f"  {body_name:30s} {joint_info['type']:8s} axis: {joint_info['axis']}")
    
    # Extract mesh data
    mesh_data = []
    print(f"\nExtracting {model.nmesh} meshes...")
    for i in range(model.nmesh):
        mesh = model.mesh(i)
        
        # Get vertices
        vert_start = model.mesh_vertadr[i]
        vert_count = model.mesh_vertnum[i]
        vertices = model.mesh_vert[vert_start:vert_start + vert_count].tolist()
        
        # Get faces
        face_start = model.mesh_faceadr[i]
        face_count = model.mesh_facenum[i]
        faces = model.mesh_face[face_start:face_start + face_count].tolist()
        
        # Find which body uses this mesh
        body_id = None
        for j in range(model.ngeom):
            if model.geom_dataid[j] == i and model.geom_type[j] == mujoco.mjtGeom.mjGEOM_MESH:
                body_id = int(model.geom_bodyid[j])
                break
        
        mesh_info = {
            'name': mesh.name,
            'vertices': vertices,
            'faces': faces,
            'body_index': body_id,
            'body_name': model.body(body_id).name if body_id is not None else None
        }
        mesh_data.append(mesh_info)
        print(f"  Mesh '{mesh.name}': {vert_count} verts, {face_count} faces -> {mesh_info['body_name']}")
    
    # Extract animation with timing
    fps = motion_data.get('fps', 30)
    dt = 1.0 / fps
    animation_frames = []
    num_frames = len(motion_data['root_pos'])
    
    print(f"\nProcessing {num_frames} frames at {fps} FPS...")
    for i in range(num_frames):
        if i % 500 == 0:
            print(f"  Frame {i}/{num_frames}")
        
        frame_data = {
            'frame_index': i,
            'time': i * dt,
            'root_position': motion_data['root_pos'][i].tolist(),
            'root_rotation': motion_data['root_rot'][i].tolist(),
            'joint_angles': motion_data['dof_pos'][i].tolist()
        }
        animation_frames.append(frame_data)
    
    output_data = {
        'metadata': {
            'num_frames': num_frames,
            'fps': fps,
            'duration': num_frames / fps,
            'source_pkl': pkl_file,
            'source_xml': robot_xml
        },
        'skeleton': skeleton_info,
        'meshes': mesh_data,
        'animation': animation_frames
    }
    
    with open(output_json, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print(f"\n✓ Saved to {output_json}")
    print(f"  Duration: {output_data['metadata']['duration']:.2f}s")
    print(f"  Bodies: {len(skeleton_info['body_names'])}")
    print(f"  Joints: {len(skeleton_info['body_to_joint_map'])}")
    print(f"  Meshes: {len(mesh_data)}")

if __name__ == "__main__":
    extract_to_json(
        pkl_file='motion_data/walk1_subject1.pkl',
        robot_xml='assets/unitree_g1/g1_mocap_29dof.xml',
        output_json='motion_data/walk1_subject1.json'
    )