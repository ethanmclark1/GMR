import bpy
import json
import mathutils

def clear_scene():
    """Remove everything"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def test_skeleton_only(json_file):
    """Test creating just the skeleton structure"""
    print("="*60)
    print("TEST 1: SKELETON STRUCTURE ONLY")
    print("="*60)
    
    # Load JSON
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    clear_scene()
    
    skeleton = data['skeleton']
    body_names = skeleton['body_names']
    parent_indices = skeleton['parent_indices']
    bind_positions = skeleton['bind_pose_global_positions']
    
    print(f"\nCreating skeleton with {len(body_names)} bodies...")
    
    # Create armature
    armature = bpy.data.armatures.new('G1_Skeleton')
    armature_obj = bpy.data.objects.new('G1_Armature', armature)
    bpy.context.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    
    # Enter edit mode
    bpy.ops.object.mode_set(mode='EDIT')
    
    bones = {}
    
    # Create bones
    for i, name in enumerate(body_names):
        if name == 'world':
            print(f"  Skipping 'world' body")
            continue
        
        bone = armature.edit_bones.new(name)
        pos = mathutils.Vector(bind_positions[i])
        
        bone.head = pos
        bone.tail = pos + mathutils.Vector((0, 0, 0.05))
        
        bones[i] = bone
        
        if i < 5:
            print(f"  Created bone [{i}] {name} at {pos}")
    
    # Set parent relationships
    print(f"\nSetting up hierarchy...")
    for i, name in enumerate(body_names):
        if name == 'world' or i not in bones:
            continue
        
        parent_idx = parent_indices[i]
        if parent_idx >= 0 and parent_idx in bones:
            bones[i].parent = bones[parent_idx]
            if i < 5:
                parent_name = body_names[parent_idx]
                print(f"  {name} -> parent: {parent_name}")
    
    # Return to object mode
    bpy.ops.object.mode_set(mode='OBJECT')
    
    print(f"\n✓ Skeleton created with {len(bones)} bones")
    
    # Save
    output_file = "test_skeleton_only.blend"
    bpy.ops.wm.save_as_mainfile(filepath=output_file)
    print(f"✓ Saved to {output_file}")
    print(f"\nOpen with: blender {output_file}")

if __name__ == "__main__":
    test_skeleton_only("motion_data/walk1_subject1.json")