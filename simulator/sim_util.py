import numpy as np
from collections import OrderedDict

import mujoco

from utils import geom, liegroup


def get_sim_info(sim):
    model, data = get_mujoco_objects(sim)
    joint_info = {}
    
    for i in range(sim.model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_info[joint_name] = i

    qpos = np.copy(data.qpos).tolist()

    return joint_info, qpos


def get_mujoco_objects(sim):
    return sim.model._model, sim.data._data


def get_col_geom(sim, object, group=0):

    model, data = get_mujoco_objects(sim)
    geom_info = {}

    for i in range(sim.model.ngeom):
        key = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if key.startswith(object.naming_prefix) and sim.model.geom(key).group == group:
            geom_info[key] = i

    return geom_info


def check_col_state(sim, geom_info_1, geom_info_2):

    model, data = get_mujoco_objects(sim)
    col_state = False
    col_info = []
    sim.forward()

    for contact in data.contact:
        if (contact.geom1 in geom_info_1.values()) and (contact.geom2 in geom_info_2.values()) or\
           (contact.geom1 in geom_info_2.values()) and (contact.geom2 in geom_info_1.values()):
            col_state = True
            col_info.append((contact.geom1, contact.geom2))

    return col_state, col_info


def get_joint_range(sim, joint_names):
    model, _ = get_mujoco_objects(sim)
    joint_ids = {
        joint_name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        for joint_name in joint_names
    }
    joint_ranges = {
        joint_name: model.jnt_range[joint_id] for joint_name, joint_id in joint_ids.items()
    }
    return joint_ranges


def get_link_jacobian(sim, link_name, whole_body_name=None, planar_body_name=None, joint_names=None, input_type='pose'):
    assert whole_body_name is not None or joint_names is not None or planar_body_name is not None
    model, data = get_mujoco_objects(sim)
    link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_XBODY, link_name)
    joint_ids = []
    if whole_body_name is not None:
        first_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, whole_body_name)
        joint_ids += [first_id + i for i in range(6)]
    if planar_body_name is not None:
        first_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, planar_body_name)
        joint_ids += [first_id, first_id+1, first_id+5]
    if joint_names is not None:
        joint_ids += [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            for joint_name in joint_names    
        ]
    J_hand = np.zeros((6, model.nv), dtype=np.float64)
    mujoco.mj_jacBody(model, data, J_hand[:3], J_hand[3:], link_id)
    return J_hand[:,joint_ids]


def get_link_pose(sim, robot, link_name):
    model, data = get_mujoco_objects(sim)
    link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_XBODY, robot.naming_prefix + link_name)
    pos = np.array(data.xpos[link_id])
    quat = np.array(data.xquat[link_id])[[1, 2, 3, 0]]
    return {"pos": pos, "quat": quat}


def get_link_twist(sim, robot, link_name):
    model, data = get_mujoco_objects(sim)
    link_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_XBODY, robot.naming_prefix + link_name)
    velp = np.copy(data.cvel[link_id])[:3]
    velr = np.copy(data.cvel[link_id])[3:]
    vel = {"pos": velp, "rpy": velr}
    return vel


def get_body_state(sim, robot):
    model, data = get_mujoco_objects(sim)
    joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, robot.naming_prefix + "root"
    )
    joint_qposadr = model.jnt_qposadr[joint_id]
    joint_qveladr = model.jnt_dofadr[joint_id]

    state = {
        "body_pose": {
            "pos": np.copy(data.qpos[joint_qposadr : joint_qposadr + 3]),
            "quat": geom.euler_to_quat(data.qpos[joint_qposadr + 3 : joint_qposadr + 6]),
        },
        "body_twist": {
            "pos": np.copy(data.qvel[joint_qveladr : joint_qveladr + 3]),
            "rpy": np.copy(data.qvel[joint_qveladr + 3 : joint_qveladr + 6]),
        },
    }

    return state


def get_link_state(sim, robot, link_names, frame_link=None):
    link_global_data = {"link_pose": OrderedDict(), "link_twist": OrderedDict()}
    for link_key in link_names:
        link_global_data["link_pose"][link_key] = get_link_pose(sim, robot, link_key)
        link_global_data["link_twist"][link_key] = get_link_twist(sim, robot, link_key)

    if frame_link==None:
        return link_global_data

    else:
        link_local_data = {"link_pose": OrderedDict(), "link_twist": OrderedDict()}

        base_pose = get_link_pose(sim, robot, frame_link)
        base_pos = base_pose["pos"]
        base_quat = base_pose["quat"]
        base_rot = geom.quat_to_rot(base_quat)
        base_iso = liegroup.RpToTrans(base_rot, base_pos)

        for link_key in link_names:
            link_global_pos = link_global_data["link_pose"][link_key]['pos']
            link_global_quat = link_global_data["link_pose"][link_key]['quat']
            link_global_rot = geom.quat_to_rot(link_global_quat)
            link_global_iso = liegroup.RpToTrans(link_global_rot, link_global_pos)
            link_local_iso = np.linalg.inv(link_global_iso) @ base_iso
            link_local_rot = link_local_iso[:3, :3]
            link_local_pos = link_local_iso[:3, 3]
            link_local_quat = geom.rot_to_quat(link_local_rot)
            link_local_data["link_pose"][link_key] = {"pos": link_local_pos, "quat": link_local_quat}
            link_local_data["link_twist"][link_key] = link_global_data["link_twist"][link_key]

    return link_local_data


def get_joint_state(sim, robot, joint_names):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map

    joint_data = {"joint_pos": OrderedDict(), "joint_vel": OrderedDict()}
    for joint_key in joint_names:
        mujoco_key = key_map["joint"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        joint_data["joint_pos"][joint_key] = data.qpos[joint_qposadr]
        joint_data["joint_vel"][joint_key] = data.qvel[joint_qveladr]

    return joint_data


def set_motor_impedance(sim, robot, command, gains):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for (joint_key, pos_des), (_, vel_des), (_, trq_des) in zip(
        command["joint_pos"].items(),
        command["joint_vel"].items(),
        command["joint_trq"].items(),
    ):
        mujoco_joint_key = key_map["joint"][joint_key]
        mujoco_actuator_key = key_map["actuator"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key
        )
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        joint_pos = data.qpos[joint_qposadr]
        joint_vel = data.qvel[joint_qveladr]
        if type(gains) == dict:
            kp_val = gains[joint_key][0]
            kd_val = gains[joint_key][1]
        else:
            kp_val = gains[0]
            kd_val = gains[1]
        data.ctrl[actuator_id] = trq_des + kp_val * (pos_des - joint_pos) + kd_val * (vel_des - joint_vel)


def set_angular_impedance(sim, robot, command, gains):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for (joint_key, quat_des), (_, vel_des), (_, trq_des) in zip(
        command["joint_pos"].items(),
        command["joint_vel"].items(),
        command["joint_trq"].items(),
    ):
        mujoco_joint_key = key_map["joint"][joint_key]
        mujoco_actuator_key = key_map["actuator"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key
        )
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        joint_pos = np.array(data.qpos[joint_qposadr+3:joint_qposadr+6])
        joint_vel = np.array(data.qvel[joint_qveladr+3:joint_qveladr+6])
        if type(gains) == dict:
            kp_val = gains[joint_key][0]
            kd_val = gains[joint_key][1]
        else:
            kp_val = gains[0]
            kd_val = gains[1]
        ang_error = geom.rot_to_euler(geom.quat_to_rot(quat_des) @ geom.euler_to_rot(joint_pos).T)
        data.ctrl[actuator_id+3:actuator_id+6] = trq_des + kp_val * ang_error + kd_val * (vel_des - joint_vel)


def set_angular_trq(sim, robot, command):
    set_linear_trq(sim, robot, command)


def set_linear_impedance(sim, robot, command, gains):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for (joint_key, pos_des), (_, vel_des), (_, trq_des) in zip(
        command["joint_pos"].items(),
        command["joint_vel"].items(),
        command["joint_trq"].items(),
    ):
        mujoco_joint_key = key_map["joint"][joint_key]
        mujoco_actuator_key = key_map["actuator"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key
        )
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        joint_pos = np.array(data.qpos[joint_qposadr:joint_qposadr+3])
        joint_vel = np.array(data.qvel[joint_qveladr:joint_qveladr+3])
        if type(gains) == dict:
            kp_val = gains[joint_key][0]
            kd_val = gains[joint_key][1]
        else:
            kp_val = gains[0]
            kd_val = gains[1]
        data.ctrl[actuator_id:actuator_id+3] = trq_des + kp_val * (pos_des - joint_pos) + kd_val * (vel_des - joint_vel)


def set_linear_trq(sim, robot, command):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for joint_key, trq_des in command["joint_trq"].items():
        mujoco_actuator_key = key_map["actuator"][joint_key]
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key
        )
        data.ctrl[actuator_id:actuator_id+3] = trq_des


def set_motor_trq(sim, robot, command):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for joint_key, trq_des in command["joint_trq"].items():
        mujoco_actuator_key = key_map["actuator"][joint_key]
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco_actuator_key
        )
        data.ctrl[actuator_id] = trq_des


def set_motor_pos(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for joint_key, pos_des in state["joint_pos"].items():
        mujoco_joint_key = key_map["joint"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        data.qpos[joint_qposadr] = pos_des


def set_motor_pos_vel(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for (joint_key, pos_des), (_, vel_des) in zip(
        state["joint_pos"].items(), state["joint_vel"].items()
    ):
        mujoco_joint_key = key_map["joint"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        data.qpos[joint_qposadr] = pos_des
        data.qvel[joint_qveladr] = vel_des


def set_linear_pos_vel(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for (joint_key, pos_des), (_, vel_des) in zip(
        state["joint_pos"].items(), state["joint_vel"].items()
    ):
        mujoco_joint_key = key_map["joint"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        data.qpos[joint_qposadr:joint_qposadr+3] = pos_des
        data.qvel[joint_qveladr:joint_qveladr+3] = vel_des


def set_angular_pos_vel(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    key_map = robot.key_map
    for (joint_key, quat_des), (_, vel_des) in zip(
        state["joint_pos"].items(), state["joint_vel"].items()
    ):
        mujoco_joint_key = key_map["joint"][joint_key]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mujoco_joint_key)
        joint_qposadr = model.jnt_qposadr[joint_id]
        joint_qveladr = model.jnt_dofadr[joint_id]
        pos_des = geom.quat_to_euler(quat_des)
        data.qpos[joint_qposadr+3:joint_qposadr+6] = pos_des
        data.qvel[joint_qveladr+3:joint_qveladr+6] = vel_des


def set_body_pos_vel(sim, robot, state):
    model, data = get_mujoco_objects(sim)
    joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, robot.naming_prefix + "root"
    )
    joint_qposadr = model.jnt_qposadr[joint_id]
    joint_qveladr = model.jnt_dofadr[joint_id]
    data.qpos[joint_qposadr : joint_qposadr + 6] = np.concatenate(
        (state["body_pose"]["pos"], geom.quat_to_euler(state["body_pose"]["quat"]))
    )
    data.qvel[joint_qveladr : joint_qveladr + 6] = np.concatenate(
        (state["body_twist"]["pos"], state["body_twist"]["rpy"])
    )
