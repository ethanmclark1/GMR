import bpy
import json
import mathutils

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def create_skeleton_with_correct_axes(json_file):
    """Create skeleton and animate using joint angles with correct axes"""
    print("="*60)
    print("FINAL: CORRECT JOINT AXES")
    print("="*60)
    
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    clear_scene()
    
    skeleton = data['skeleton']
    body_names = skeleton['body_names']
    parent_indices = skeleton['parent_indices']
    bind_positions = skeleton['bind_pose_positions']
    body_to_joint = skeleton['body_to_joint_map']
    animation = data['animation']
    metadata = data['metadata']
    
    test_frames = min(300, len(animation))  # Test with 300 frames
    fps = metadata['fps']
    
    print(f"\nCreating skeleton...")
    
    # Create armature
    armature = bpy.data.armatures.new('G1_Skeleton')
    armature_obj = bpy.data.objects.new('G1_Armature', armature)
    bpy.context.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    
    bpy.ops.object.mode_set(mode='EDIT')
    
    bones = {}
    
    # Create bones
    for i, name in enumerate(body_names):
        if name == 'world':
            continue
        
        bone = armature.edit_bones.new(name)
        pos = mathutils.Vector(bind_positions[i])
        
        bone.head = pos
        bone.tail = pos + mathutils.Vector((0, 0, 0.05))
        
        bones[i] = bone
    
    # Set parents
    for i, name in enumerate(body_names):
        if name == 'world' or i not in bones:
            continue
        
        parent_idx = parent_indices[i]
        if parent_idx >= 0 and parent_idx in bones:
            bones[i].parent = bones[parent_idx]
    
    # Adjust tails
    for i, name in enumerate(body_names):
        if name == 'world' or i not in bones:
            continue
        
        bone = bones[i]
        bone_pos = mathutils.Vector(bind_positions[i])
        
        children = [j for j, p in enumerate(parent_indices) if p == i and j in bones]
        
        if children:
            child_pos = mathutils.Vector(bind_positions[children[0]])
            direction = child_pos - bone_pos
            if direction.length > 0.001:
                bone.tail = bone_pos + direction * 0.5
        else:
            if bone.parent:
                parent_dir = bone.head - bone.parent.head
                if parent_dir.length > 0.001:
                    bone.tail = bone.head + parent_dir.normalized() * 0.05
    
    bpy.ops.object.mode_set(mode='OBJECT')
    
    print(f"✓ Skeleton created with {len(bones)} bones")
    
    # Setup scene
    bpy.context.scene.render.fps = fps
    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = test_frames
    
    print(f"\nApplying animation ({test_frames} frames)...")
    
    pose_bones = armature_obj.pose.bones
    
    # Apply animation
    for frame_data in animation[:test_frames]:
        frame_idx = frame_data['frame_index'] + 1
        root_pos = frame_data['root_position']
        root_rot = frame_data['root_rotation']
        joint_angles = frame_data['joint_angles']
        
        if frame_idx % 50 == 1:
            print(f"  Frame {frame_idx}/{test_frames}")
        
        bpy.context.scene.frame_set(frame_idx)
        
        # Set pelvis (root)
        pelvis_bone = pose_bones['pelvis']
        pelvis_bone.location = mathutils.Vector(root_pos)
        pelvis_bone.rotation_mode = 'QUATERNION'
        pelvis_bone.rotation_quaternion = mathutils.Quaternion((root_rot[0], root_rot[1], root_rot[2], root_rot[3]))
        pelvis_bone.keyframe_insert(data_path="location", frame=frame_idx)
        pelvis_bone.keyframe_insert(data_path="rotation_quaternion", frame=frame_idx)
        
        # Set other bones using joint angles with correct axes
        for body_name, joint_info in body_to_joint.items():
            if body_name == 'pelvis':
                continue
            
            qpos_addr = joint_info['qpos_address']
            axis = joint_info['axis']
            
            # Map qpos address to joint_angles index
            joint_angle_idx = qpos_addr - 7
            
            if joint_angle_idx < 0 or joint_angle_idx >= len(joint_angles):
                continue
            
            angle = joint_angles[joint_angle_idx]
            
            bone = pose_bones[body_name]
            bone.rotation_mode = 'XYZ'
            
            # Apply rotation around the correct axis
            if axis == [1.0, 0.0, 0.0]:  # X-axis (roll)
                bone.rotation_euler = (angle, 0, 0)
            elif axis == [0.0, 1.0, 0.0]:  # Y-axis (pitch)
                bone.rotation_euler = (0, angle, 0)
            elif axis == [0.0, 0.0, 1.0]:  # Z-axis (yaw)
                bone.rotation_euler = (0, 0, angle)
            
            bone.keyframe_insert(data_path="rotation_euler", frame=frame_idx)
    
    bpy.context.scene.frame_set(1)
    
    print(f"✓ Animation applied with correct joint axes")
    print(f"\nVERIFY:")
    print(f"  1. Press SPACE to play animation")
    print(f"  2. Robot should walk naturally at correct height")
    print(f"  3. All joints should move smoothly")
    
    output_file = "g1_animation_final.blend"
    bpy.ops.wm.save_as_mainfile(filepath=output_file)
    print(f"\n✓ Saved to {output_file}")
    print(f"\nIf animation looks correct, you can:")
    print(f"  1. File > Export > FBX (.fbx)")
    print(f"  2. Enable 'Bake Animation' in export settings")
    print(f"  3. Import into Unreal Engine")

if __name__ == "__main__":
    create_skeleton_with_correct_axes("motion_data/walk1_with_axes.json")