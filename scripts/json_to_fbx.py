import bpy
import json
import math
import mathutils
import os
import glob

def clear_scene():
    """Remove all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def quaternion_to_matrix(quat):
    """Convert quaternion [w, x, y, z] to rotation matrix"""
    return mathutils.Quaternion(quat).to_matrix().to_4x4()

def create_armature_from_skeleton(skeleton_data):
    """Create Blender armature from skeleton data"""
    
    # Create armature object
    armature = bpy.data.armatures.new('Skeleton')
    armature_obj = bpy.data.objects.new('Armature', armature)
    bpy.context.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    
    # Enter edit mode to create bones
    bpy.ops.object.mode_set(mode='EDIT')
    
    body_names = skeleton_data['body_names']
    parent_indices = skeleton_data['parent_indices']
    bind_positions = skeleton_data['bind_pose_global_positions']
    bind_rotations = skeleton_data['bind_pose_global_rotations']
    
    bones = {}
    
    # Create bones
    for i, name in enumerate(body_names):
        if name == 'world':  # Skip world body
            continue
            
        bone = armature.edit_bones.new(name)
        pos = mathutils.Vector(bind_positions[i])
        
        # Set bone head at body position
        bone.head = pos
        
        # Set bone tail (pointing towards child or slightly offset)
        bone.tail = pos + mathutils.Vector((0, 0.1, 0))
        
        bones[i] = bone
    
    # Set up parent relationships
    for i, name in enumerate(body_names):
        if name == 'world' or i not in bones:
            continue
            
        parent_idx = parent_indices[i]
        if parent_idx >= 0 and parent_idx in bones:
            bones[i].parent = bones[parent_idx]
    
    # Return to object mode
    bpy.ops.object.mode_set(mode='OBJECT')
    
    return armature_obj

def apply_animation(armature_obj, animation_data, metadata):
    """Apply animation frames to armature"""
    
    fps = metadata.get('fps', 30)
    num_frames = metadata['num_frames']
    
    # Set up scene
    bpy.context.scene.render.fps = fps
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = num_frames - 1
    
    # Get pose bones
    pose_bones = armature_obj.pose.bones
    
    print(f"Applying {num_frames} frames of animation...")
    
    # Animate each frame
    for frame_data in animation_data:
        frame_idx = frame_data['frame_index']
        positions = frame_data['positions']
        rotations = frame_data['rotations']
        
        if frame_idx % 100 == 0:
            print(f"  Frame {frame_idx}/{num_frames}")
        
        bpy.context.scene.frame_set(frame_idx)
        
        # Apply transforms to each bone
        for i, bone in enumerate(pose_bones):
            if i >= len(positions):
                continue
                
            # Convert quaternion (MuJoCo format: [w,x,y,z])
            quat = rotations[i]
            rotation = mathutils.Quaternion((quat[0], quat[1], quat[2], quat[3]))
            
            # Set pose bone rotation
            bone.rotation_quaternion = rotation
            
            # Set location for root bone
            if i == 0 or bone.parent is None:
                bone.location = mathutils.Vector(positions[i])
            
            # Insert keyframes
            bone.keyframe_insert(data_path="rotation_quaternion", frame=frame_idx)
            if i == 0 or bone.parent is None:
                bone.keyframe_insert(data_path="location", frame=frame_idx)
    
    print("✓ Animation applied")

def normalize_name(name):
    """Normalize bone/mesh names for matching"""
    # Convert various naming conventions to match
    return name.lower().replace('_link', '').replace('-', '_').replace(' ', '_')

def import_meshes(mesh_dir, armature_obj, body_names):
    """Import all STL meshes and attach to corresponding bones"""
    
    if not os.path.exists(mesh_dir):
        print(f"⚠ Mesh directory not found: {mesh_dir}")
        return []
    
    # Get all STL files
    stl_files = glob.glob(os.path.join(mesh_dir, "*.STL")) + \
                glob.glob(os.path.join(mesh_dir, "*.stl"))
    
    if not stl_files:
        print(f"⚠ No STL files found in {mesh_dir}")
        return []
    
    print(f"\nFound {len(stl_files)} mesh files")
    print(f"Importing and attaching meshes to bones...")
    
    # Create name mapping for bones
    bone_map = {}
    for bone in armature_obj.data.bones:
        normalized = normalize_name(bone.name)
        bone_map[normalized] = bone.name
    
    imported_meshes = []
    matched_count = 0
    
    for stl_file in stl_files:
        filename = os.path.basename(stl_file)
        mesh_name = os.path.splitext(filename)[0]
        normalized_mesh = normalize_name(mesh_name)
        
        # Import STL
        bpy.ops.import_mesh.stl(filepath=stl_file)
        mesh_obj = bpy.context.selected_objects[0]
        mesh_obj.name = mesh_name
        
        # Try to find matching bone
        matching_bone = None
        
        # Direct match
        if normalized_mesh in bone_map:
            matching_bone = bone_map[normalized_mesh]
        else:
            # Fuzzy match - check if bone name contains mesh name or vice versa
            for norm_bone, bone_name in bone_map.items():
                if normalized_mesh in norm_bone or norm_bone in normalized_mesh:
                    matching_bone = bone_name
                    break
        
        if matching_bone:
            # Parent mesh to specific bone
            mesh_obj.parent = armature_obj
            mesh_obj.parent_type = 'BONE'
            mesh_obj.parent_bone = matching_bone
            matched_count += 1
            print(f"  ✓ {mesh_name} → {matching_bone}")
        else:
            # Just parent to armature without specific bone
            mesh_obj.parent = armature_obj
            print(f"  ⚠ {mesh_name} (no matching bone found)")
        
        imported_meshes.append(mesh_obj)
    
    print(f"\n✓ Imported {len(imported_meshes)} meshes ({matched_count} matched to bones)")
    return imported_meshes

def json_to_fbx(json_file, output_fbx, mesh_dir=None):
    """Main conversion function"""
    
    print(f"\n{'='*60}")
    print(f"Converting JSON to FBX")
    print(f"{'='*60}\n")
    
    # Load JSON data
    print(f"Loading {json_file}...")
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # Clear scene
    clear_scene()
    
    # Create armature
    print("Creating skeleton...")
    armature_obj = create_armature_from_skeleton(data['skeleton'])
    
    # Import meshes if directory provided
    if mesh_dir:
        body_names = data['skeleton']['body_names']
        import_meshes(mesh_dir, armature_obj, body_names)
    
    # Apply animation
    apply_animation(armature_obj, data['animation'], data['metadata'])
    
    # Export to FBX
    print(f"\nExporting to {output_fbx}...")
    bpy.ops.export_scene.fbx(
        filepath=output_fbx,
        use_selection=False,
        bake_anim=True,
        bake_anim_use_all_bones=True,
        bake_anim_use_nla_strips=False,
        bake_anim_use_all_actions=False,
        add_leaf_bones=False,
        mesh_smooth_type='FACE'
    )
    
    print(f"\n{'='*60}")
    print(f"✓ Successfully exported to {output_fbx}")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    # Configuration
    json_file = "retargeted/walk1_subject1.json"
    output_fbx = "retargeted/walk1_subject1.fbx"
    mesh_dir = "assets/unitree_g1/meshes"
    
    # Convert
    json_to_fbx(json_file, output_fbx, mesh_dir)