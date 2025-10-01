import bpy
import json
import math
import mathutils
import os
import glob
import sys

def clear_scene():
    """Remove all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # Clear orphan data
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in bpy.data.materials:
        if block.users == 0:
            bpy.data.materials.remove(block)

def create_armature_from_skeleton(skeleton_data):
    """Create Blender armature from skeleton data"""
    
    # Create armature object
    armature = bpy.data.armatures.new('G1_Skeleton')
    armature_obj = bpy.data.objects.new('G1_Armature', armature)
    bpy.context.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    
    # Enter edit mode to create bones
    bpy.ops.object.mode_set(mode='EDIT')
    
    body_names = skeleton_data['body_names']
    parent_indices = skeleton_data['parent_indices']
    bind_positions = skeleton_data['bind_pose_global_positions']
    
    bones = {}
    
    print(f"Creating {len(body_names)} bones...")
    
    # Create bones
    for i, name in enumerate(body_names):
        if name == 'world':  # Skip world body
            continue
            
        bone = armature.edit_bones.new(name)
        pos = mathutils.Vector(bind_positions[i])
        
        # Set bone head at body position
        bone.head = pos
        
        # Set bone tail - smaller default tail
        bone.tail = pos + mathutils.Vector((0, 0, 0.05))
        
        bones[i] = bone
    
    # Set up parent relationships and adjust tails to point at children
    for i, name in enumerate(body_names):
        if name == 'world' or i not in bones:
            continue
            
        parent_idx = parent_indices[i]
        if parent_idx >= 0 and parent_idx in bones:
            bones[i].parent = bones[parent_idx]
            
            # Make parent bone point towards this child
            parent_bone = bones[parent_idx]
            child_pos = mathutils.Vector(bind_positions[i])
            parent_pos = mathutils.Vector(bind_positions[parent_idx])
            
            direction = child_pos - parent_pos
            if direction.length > 0.01:
                parent_bone.tail = parent_pos + direction * 0.5
    
    # Return to object mode
    bpy.ops.object.mode_set(mode='OBJECT')
    
    print(f"✓ Created {len(bones)} bones")
    return armature_obj

def apply_animation(armature_obj, animation_data, metadata):
    """Apply animation frames to armature"""
    
    fps = metadata.get('fps', 30)
    num_frames = metadata['num_frames']
    
    # Set up scene
    bpy.context.scene.render.fps = fps
    bpy.context.scene.frame_start = 1
    bpy.context.scene.frame_end = num_frames
    
    # Get pose bones
    pose_bones = armature_obj.pose.bones
    body_names = [bone.name for bone in armature_obj.data.bones]
    
    print(f"\nApplying {num_frames} frames of animation...")
    
    # Animate each frame
    for frame_data in animation_data:
        frame_idx = frame_data['frame_index'] + 1  # Blender frames start at 1
        positions = frame_data['positions']
        rotations = frame_data['rotations']
        
        if frame_idx % 100 == 1 or frame_idx == num_frames:
            print(f"  Frame {frame_idx}/{num_frames}")
        
        bpy.context.scene.frame_set(frame_idx)
        
        # Apply transforms to each bone
        for bone_name in body_names:
            if bone_name == 'world':
                continue
            
            try:
                bone_idx = body_names.index(bone_name)
            except ValueError:
                continue
            
            if bone_idx >= len(positions):
                continue
            
            bone = pose_bones[bone_name]
            
            # Convert quaternion (MuJoCo uses [w,x,y,z])
            quat = rotations[bone_idx]
            rotation = mathutils.Quaternion((quat[0], quat[1], quat[2], quat[3]))
            
            # Set pose bone rotation
            bone.rotation_mode = 'QUATERNION'
            bone.rotation_quaternion = rotation
            
            # Set location for root bone only
            if bone.parent is None:
                bone.location = mathutils.Vector(positions[bone_idx])
                bone.keyframe_insert(data_path="location", frame=frame_idx)
            
            # Insert keyframes
            bone.keyframe_insert(data_path="rotation_quaternion", frame=frame_idx)
    
    print("✓ Animation applied successfully")

def normalize_name(name):
    """Normalize bone/mesh names for matching"""
    return name.lower().replace('_link', '').replace('-', '_').replace(' ', '_')

def import_stl_file(filepath):
    """Import STL file using the correct operator for Blender version"""
    try:
        bpy.ops.wm.stl_import(filepath=filepath)
    except AttributeError:
        try:
            bpy.ops.import_mesh.stl(filepath=filepath)
        except AttributeError:
            return None
    
    if bpy.context.selected_objects:
        return bpy.context.selected_objects[0]
    return None

def import_meshes(mesh_dir, armature_obj, skeleton_data):
    """Import all STL meshes and attach to corresponding bones with proper transforms"""
    
    if not os.path.exists(mesh_dir):
        print(f"⚠ Mesh directory not found: {mesh_dir}")
        return []
    
    # Get all STL files and remove duplicates
    stl_files = glob.glob(os.path.join(mesh_dir, "*.STL")) + \
                glob.glob(os.path.join(mesh_dir, "*.stl"))
    
    # Remove duplicates based on filename
    seen_names = set()
    unique_stl_files = []
    for stl_file in stl_files:
        filename = os.path.basename(stl_file)
        if filename.lower() not in seen_names:
            seen_names.add(filename.lower())
            unique_stl_files.append(stl_file)
    
    if not unique_stl_files:
        print(f"⚠ No STL files found in {mesh_dir}")
        return []
    
    print(f"\nFound {len(unique_stl_files)} unique mesh files")
    print(f"Importing and attaching meshes to bones...")
    
    # Create name and position mapping for bones
    bone_map = {}
    bone_positions = {}
    body_names = skeleton_data['body_names']
    bind_positions = skeleton_data['bind_pose_global_positions']
    local_offsets = skeleton_data['local_offsets']
    
    for i, bone_name in enumerate(body_names):
        if bone_name == 'world':
            continue
        normalized = normalize_name(bone_name)
        bone_map[normalized] = bone_name
        bone_positions[bone_name] = {
            'global_pos': mathutils.Vector(bind_positions[i]),
            'local_offset': mathutils.Vector(local_offsets[i])
        }
    
    imported_meshes = []
    matched_count = 0
    
    # Set armature to pose mode temporarily
    bpy.context.view_layer.objects.active = armature_obj
    bpy.ops.object.mode_set(mode='OBJECT')
    
    for idx, stl_file in enumerate(unique_stl_files):
        if idx % 20 == 0:
            print(f"  Processing mesh {idx+1}/{len(unique_stl_files)}...")
            
        filename = os.path.basename(stl_file)
        mesh_name = os.path.splitext(filename)[0]
        normalized_mesh = normalize_name(mesh_name)
        
        # Import STL
        mesh_obj = import_stl_file(stl_file)
        
        if mesh_obj is None:
            continue
            
        mesh_obj.name = mesh_name
        
        # Try to find matching bone
        matching_bone = None
        
        # Direct match
        if normalized_mesh in bone_map:
            matching_bone = bone_map[normalized_mesh]
        else:
            # Fuzzy match
            for norm_bone, bone_name in bone_map.items():
                if normalized_mesh in norm_bone or norm_bone in normalized_mesh:
                    matching_bone = bone_name
                    break
        
        if matching_bone:
            # Clear any existing transforms on the mesh
            mesh_obj.location = (0, 0, 0)
            mesh_obj.rotation_euler = (0, 0, 0)
            mesh_obj.scale = (1, 1, 1)
            
            # Parent mesh to specific bone with automatic weights disabled
            mesh_obj.parent = armature_obj
            mesh_obj.parent_type = 'BONE'
            mesh_obj.parent_bone = matching_bone
            
            # Set the mesh to use the bone's pose transform
            # This keeps the mesh in its original position relative to the bone
            mesh_obj.matrix_parent_inverse = armature_obj.matrix_world.inverted()
            
            matched_count += 1
        else:
            # Just parent to armature at world origin
            mesh_obj.parent = armature_obj
        
        imported_meshes.append(mesh_obj)
    
    print(f"✓ Imported {len(imported_meshes)} meshes ({matched_count} matched to bones)")
    return imported_meshes

def json_to_blend(json_file, output_blend, mesh_dir=None):
    """Main conversion function - saves as .blend file"""
    
    print(f"\n{'='*60}")
    print(f"Converting JSON to Blender file")
    print(f"{'='*60}\n")
    
    # Load JSON data
    print(f"Loading {json_file}...")
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"✗ Error loading JSON: {e}")
        return False
    
    # Clear scene
    clear_scene()
    
    # Create armature
    print("\nCreating skeleton...")
    try:
        armature_obj = create_armature_from_skeleton(data['skeleton'])
    except Exception as e:
        print(f"✗ Error creating skeleton: {e}")
        return False
    
    # Import meshes if directory provided
    if mesh_dir:
        try:
            import_meshes(mesh_dir, armature_obj, data['skeleton'])
        except Exception as e:
            print(f"⚠ Warning: Mesh import had issues: {e}")
            import traceback
            traceback.print_exc()
    
    # Apply animation
    try:
        apply_animation(armature_obj, data['animation'], data['metadata'])
    except Exception as e:
        print(f"✗ Error applying animation: {e}")
        return False
    
    # Set viewport to frame 1 and reset pose
    bpy.context.scene.frame_set(1)
    
    # Make sure output directory exists
    output_dir = os.path.dirname(output_blend)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Save as .blend file
    print(f"\nSaving to {output_blend}...")
    try:
        bpy.ops.wm.save_as_mainfile(filepath=output_blend)
    except Exception as e:
        print(f"✗ Error saving blend file: {e}")
        return False
    
    # Verify file was created
    if os.path.exists(output_blend):
        file_size = os.path.getsize(output_blend)
        print(f"\n{'='*60}")
        print(f"✓ Successfully saved to {output_blend}")
        print(f"  File size: {file_size / 1024 / 1024:.2f} MB")
        print(f"\n📝 Next steps:")
        print(f"  1. Open in Blender: blender {output_blend}")
        print(f"  2. Press SPACE to play animation")
        print(f"  3. Verify meshes and animation look correct")
        print(f"  4. Export: File > Export > FBX (.fbx)")
        print(f"{'='*60}\n")
        return True
    else:
        print(f"\n✗ Blend file was not created!")
        return False

if __name__ == "__main__":
    # Configuration
    json_file = "motion_data/walk1_subject1.json"
    output_blend = "motion_data/walk1_subject1.blend"
    mesh_dir = "assets/unitree_g1/meshes"
    
    # Convert
    success = json_to_blend(json_file, output_blend, mesh_dir)
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)