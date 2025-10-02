#!/usr/bin/env python3

import bpy
import json
from mathutils import Vector, Quaternion, Matrix

def clear_scene():
    """Remove all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    
    for block in bpy.data.armatures:
        if block.users == 0:
            bpy.data.armatures.remove(block)

def load_json(json_path):
    with open(json_path, 'r') as f:
        return json.load(f)

def compute_world_transforms(skeleton):
    """Compute world-space position and rotation for each body"""
    body_names = skeleton['body_names']
    parent_indices = skeleton['parent_indices']
    local_positions = skeleton['bind_pose_local_positions']
    local_rotations = skeleton['bind_pose_local_rotations']
    
    num_bodies = len(body_names)
    world_positions = [None] * num_bodies
    world_rotations = [None] * num_bodies
    
    for i in range(num_bodies):
        parent_idx = parent_indices[i]
        local_pos = Vector(local_positions[i])
        local_quat = Quaternion(local_rotations[i])
        
        if parent_idx < 0 or parent_idx == i:
            world_positions[i] = local_pos
            world_rotations[i] = local_quat
        else:
            parent_pos = world_positions[parent_idx]
            parent_quat = world_rotations[parent_idx]
            world_positions[i] = parent_pos + parent_quat @ local_pos
            world_rotations[i] = parent_quat @ local_quat
    
    return world_positions, world_rotations

def create_armature(data):
    """Create armature from skeleton data"""
    skeleton = data['skeleton']
    
    print("\n Creating Armature...")
    
    world_positions, world_rotations = compute_world_transforms(skeleton)
    
    armature = bpy.data.armatures.new('G1_Skeleton')
    armature_obj = bpy.data.objects.new('G1_Armature', armature)
    bpy.context.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    
    bpy.ops.object.mode_set(mode='EDIT')
    
    body_names = skeleton['body_names']
    parent_indices = skeleton['parent_indices']
    
    edit_bones = {}
    
    for i, body_name in enumerate(body_names):
        if body_name == 'world':
            edit_bones[i] = None
            continue
        
        bone = armature.edit_bones.new(body_name)
        edit_bones[i] = bone
        bone.head = world_positions[i]
        bone.tail = world_positions[i] + Vector((0, 0, 0.05))
        
        parent_idx = parent_indices[i]
        if parent_idx > 0 and edit_bones.get(parent_idx) is not None:
            bone.parent = edit_bones[parent_idx]
    
    # Adjust bone tails
    for i, body_name in enumerate(body_names):
        if body_name == 'world' or edit_bones[i] is None:
            continue
        
        bone = edit_bones[i]
        children = [edit_bones[j] for j in range(len(body_names)) 
                   if parent_indices[j] == i and edit_bones.get(j) is not None]
        
        if children:
            if len(children) == 1:
                bone.tail = children[0].head
            else:
                avg_child_pos = sum([child.head for child in children], Vector()) / len(children)
                bone.tail = avg_child_pos
            
            bone_vec = bone.tail - bone.head
            if bone_vec.length < 0.01:
                if bone.parent:
                    parent_dir = (bone.head - bone.parent.head).normalized()
                    bone.tail = bone.head + parent_dir * 0.05
                else:
                    bone.tail = bone.head + Vector((0, 0, 0.05))
        else:
            if bone.parent:
                parent_dir = (bone.head - bone.parent.head).normalized()
                bone.tail = bone.head + parent_dir * 0.05
            else:
                world_quat = world_rotations[i]
                tail_dir = world_quat @ Vector((0, 0.05, 0))
                bone.tail = bone.head + tail_dir
    
    bpy.ops.object.mode_set(mode='OBJECT')
    
    print(f"Created {len([b for b in edit_bones.values() if b is not None])} bones")
    
    return armature_obj, world_positions, world_rotations

def create_meshes(data, armature_obj, world_positions, world_rotations):
    """Create mesh objects with correct orientation"""
    meshes_data = data.get('meshes', [])
    skeleton = data['skeleton']
    body_names = skeleton['body_names']
    
    print(f"\nCreating {len(meshes_data)} meshes...")
    
    # Create a mapping from body name to index
    body_name_to_idx = {name: i for i, name in enumerate(body_names)}
    
    mesh_objects = []
    
    for mesh_data in meshes_data:
        mesh_name = mesh_data['name']
        vertices = mesh_data['vertices']
        faces = mesh_data['faces']
        body_name = mesh_data.get('body_name')
        
        if not body_name or body_name not in armature_obj.pose.bones:
            print(f"{mesh_name:30s} -> NO BONE FOUND")
            continue
        
        # Create mesh
        mesh = bpy.data.meshes.new(mesh_name)
        mesh.from_pydata(vertices, [], faces)
        mesh.update()
        
        # Create object
        mesh_obj = bpy.data.objects.new(mesh_name, mesh)
        bpy.context.collection.objects.link(mesh_obj)
        
        # Get body's world transform from MuJoCo bind pose
        body_idx = body_name_to_idx[body_name]
        body_world_pos = world_positions[body_idx]
        body_world_rot = world_rotations[body_idx]
        
        # Set mesh world transform to match MuJoCo body transform
        # This positions the mesh exactly where it should be in world space
        body_world_matrix = Matrix.Translation(body_world_pos) @ body_world_rot.to_matrix().to_4x4()
        mesh_obj.matrix_world = body_world_matrix
        
        # Now parent to bone - Blender will automatically calculate the correct offset
        # We need to keep the transform, so we use keep_transform=True
        mesh_obj.parent = armature_obj
        mesh_obj.parent_type = 'BONE'
        mesh_obj.parent_bone = body_name
        
        # CRITICAL: After parenting, we need to preserve the world transform
        # Re-apply the world matrix to ensure it stays in place
        mesh_obj.matrix_world = body_world_matrix
        
        print(f"   ✓ {mesh_name:30s} -> {body_name} ({len(vertices)} verts)")
        
        mesh_objects.append(mesh_obj)
    
    return mesh_objects

def main():
    JSON_FILE = 'motion_data/walk1_subject1.json'
    
    print("="*60)
    print("Mesh Orientation")
    print("="*60)
    
    # Setup
    clear_scene()
    data = load_json(JSON_FILE)
    
    # Create armature
    armature_obj, world_positions, world_rotations = create_armature(data)
    
    # Create meshes
    mesh_objects = create_meshes(data, armature_obj, world_positions, world_rotations)
    
    print(f"\nTotal objects created:")
    print(f"   Armature: 1")
    print(f"   Meshes: {len(mesh_objects)}")
    
    # Save
    output_blend = 'motion_data/g1_with_meshes.blend'
    bpy.ops.wm.save_as_mainfile(filepath=output_blend)
    print(f"\nSaved to: {output_blend}")
    print("   In Blender: Select armature, go to Pose Mode (Ctrl+Tab)")
    print("   Rotate a bone (R) to test if meshes follow correctly")

if __name__ == "__main__":
    main()