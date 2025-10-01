import mujoco

def inspect_mujoco_model(robot_xml):
    """Inspect the MuJoCo model structure"""
    print("\n" + "="*60)
    print("MUJOCO MODEL INSPECTION")
    print("="*60)
    
    model = mujoco.MjModel.from_xml_path(robot_xml)
    data = mujoco.MjData(model)
    
    print(f"\n1. Model Statistics:")
    print(f"   Bodies: {model.nbody}")
    print(f"   Joints: {model.njnt}")
    print(f"   DOFs: {model.nv}")
    print(f"   qpos size: {model.nq}")
    
    print(f"\n2. Body hierarchy:")
    for i in range(model.nbody):
        body = model.body(i)
        parent_id = model.body_parentid[i]
        parent_name = model.body(parent_id).name if parent_id >= 0 else "None"
        print(f"   [{i}] {body.name:30s} parent: {parent_name:20s} parent_id: {parent_id}")
    
    print(f"\n3. Joint information:")
    for i in range(model.njnt):
        joint = model.joint(i)
        body_id = model.jnt_bodyid[i]
        body_name = model.body(body_id).name
        qpos_adr = model.jnt_qposadr[i]
        dof_adr = model.jnt_dofadr[i]
        print(f"   [{i}] {joint.name:30s} body: {body_name:20s} qpos_adr: {qpos_adr} dof_adr: {dof_adr}")
    
    print(f"\n4. Body local transforms (from XML):")
    for i in range(min(10, model.nbody)):  # First 10 bodies
        body = model.body(i)
        print(f"   {body.name:30s} pos: {model.body_pos[i]} quat: {model.body_quat[i]}")
    
    print(f"\n5. qpos breakdown:")
    print(f"   Total qpos size: {model.nq}")
    print(f"   Expected: 3 (root_pos) + 4 (root_rot quat) + {model.nv - 6} (joint angles)")
    
    # Set to zero and get bind pose
    print(f"\n6. Bind pose (qpos=0):")
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    
    print(f"   Root body global position: {data.xpos[0]}")
    print(f"   Root body global rotation: {data.xquat[0]}")
    
    for i in range(min(5, model.nbody)):
        body = model.body(i)
        print(f"   {body.name:30s} xpos: {data.xpos[i]} xquat: {data.xquat[i]}")
    
    return model, data

if __name__ == "__main__":
    model, data = inspect_mujoco_model('assets/unitree_g1/g1_mocap_29dof.xml')