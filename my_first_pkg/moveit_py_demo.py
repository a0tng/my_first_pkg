import argparse
import copy
import math
import os
import sys
import time

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String, Bool
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Constraints, PositionConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from my_first_pkg import kit_moveit_config

LOG = get_logger('moveit_py_demo')

HOME = [0.0, -1.501, 0.8029, 0.6981]         # pick_demo POSES['home'] = SRDF home          
READY1 = [0.0698, 0.5934, -0.9076, 0.2967]
APPROACH1 = [0.0698,  0.5585, -0.4189, -0.1571] # pick_demo POSES['approach']
LIFT = [0.0,  -0.6632, 0.3665, 0.3316]
READY2 = [0.0, 0.5934, -0.9076, 0.2967]
APPROACH2 = [0.0, 0.5585, -0.4189, -0.1571]
FRAME = 'base_footprint'                   # pose 목표 기준 좌표계 (모델 기준 좌표계)
EE = 'end_effector_link'

BOX_POS = (0.14, 0.0, 0.30)                # 6단계 상자 위치 [m] (base_footprint 기준, 헤드리스 검증값)
BOX_SIZE = [0.06, 0.30, 0.06]              # 6단계 상자 크기 [m]


# ---------------------------------------------------------------- 공통 함수
def current(robot, group='arm'):
    with robot.get_planning_scene_monitor().read_only() as scene:
        return list(scene.current_state.get_joint_group_positions(group))


def fmt(q):
    return '[' + ', '.join(f'{v:+.3f}' for v in q) + ']'


def log_joints(robot, label):
    LOG.info(f'  [{label}] 현재 arm = {fmt(current(robot))}, gripper = {fmt(current(robot, "gripper"))}')


def plan_exec(robot, comp, label, execute=True, attempts=1):
    """계획 → (선택) 실행. 성공하면 계획 결과, 실패하면 None."""
    t0 = time.time()
    res = None
    for k in range(attempts):                      # 샘플링 기반 계획은 확률적 — 필요하면 재시도
        res = comp.plan()
        if res:
            break
    ok = bool(res)
    t_plan = time.time() - t0
    n = len(res.trajectory.get_robot_trajectory_msg().joint_trajectory.points) if ok else 0
    ex = robot.execute(comp.planning_group_name, res.trajectory, blocking=True).status if (ok and execute) else None
    LOG.info(f'[{label}] plan={"OK" if ok else "FAIL"} (try {k + 1}/{attempts}, plan {t_plan:.2f}s, {n} pts) '
             f'exec={ex if execute else "(계획만)"}')
    if not ok:
        raise RuntimeError(f'[{label}] 계획 실패')
    if execute and ex != 'SUCCEEDED':
        raise RuntimeError(f'[{label}] 실행 실패: {ex}')
    return res

def joint_state(model, q):
    """팔 관절값 q의 RobotState. 생성 직후 set_to_default_values() 필수 (안 하면 그리퍼·바퀴에 쓰레기 값)."""
    s = RobotState(model)
    s.set_to_default_values()
    s.set_joint_group_positions('arm', q)
    s.update()
    return s


def go_named(robot, arm, name, label):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    return plan_exec(robot, arm, label)


# ---------------------------------------------------------------- 단계
def home(ctx):
    """1 named target: SRDF의 'home'."""
    robot, arm = ctx['robot'], ctx['arm']
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['home_state'])
    plan_exec(robot, arm, 'home')
    log_joints(robot, 'home')

def ready1(ctx):
    """2 관절 목표: pick_demo의 approach와 같은 관절값."""
    robot, arm = ctx['robot'], ctx['arm']
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['ready1_state'])
    plan_exec(robot, arm, 'ready1')
    log_joints(robot, 'ready1')

def approach1(ctx):
    """2 관절 목표: pick_demo의 approach와 같은 관절값."""
    robot, arm = ctx['robot'], ctx['arm']
    ee = ctx['approach1_ee']
    LOG.info(f'  approach1 FK: {EE} = ({ee.position.x:.3f}, {ee.position.y:.3f}, {ee.position.z:.3f}) m, '
             f'q = ({ee.orientation.x:.3f}, {ee.orientation.y:.3f}, {ee.orientation.z:.3f}, {ee.orientation.w:.3f})')
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['approach1_state'])
    plan_exec(robot, arm, 'joint approach1')
    log_joints(robot, 'joint approach1')

def ready2(ctx):
    """2 관절 목표: pick_demo의 approach와 같은 관절값."""
    robot, arm = ctx['robot'], ctx['arm']
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['ready2_state'])
    plan_exec(robot, arm, 'ready2')
    log_joints(robot, 'ready2')

def lift(ctx):
    """2 관절 목표: pick_demo의 approach와 같은 관절값."""
    robot, arm = ctx['robot'], ctx['arm']
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['lift_state'])
    plan_exec(robot, arm, 'lift')
    log_joints(robot, 'lift')

def approach2(ctx):
    """2 관절 목표: pick_demo의 approach와 같은 관절값."""
    robot, arm = ctx['robot'], ctx['arm']
    ee = ctx['approach2_ee']
    LOG.info(f'  approach2 FK: {EE} = ({ee.position.x:.3f}, {ee.position.y:.3f}, {ee.position.z:.3f}) m, '
             f'q = ({ee.orientation.x:.3f}, {ee.orientation.y:.3f}, {ee.orientation.z:.3f}, {ee.orientation.w:.3f})')
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['approach2_state'])
    plan_exec(robot, arm, 'joint approach2')
    log_joints(robot, 'joint approach2')

def gripper_open(ctx):
    """5 그리퍼 open (SRDF named state, GripperCommand 액션)."""
    robot, grip = ctx['robot'], ctx['grip']
    grip.set_start_state_to_current_state()
    grip.set_goal_state(configuration_name='open')
    plan_exec(robot, grip, '5 gripper open')
    log_joints(robot, 'gripper open')

def gripper_close(ctx):
    """5 그리퍼 close (SRDF named state, GripperCommand 액션)."""
    robot, grip = ctx['robot'], ctx['grip']
    grip.set_start_state_to_current_state()
    grip.set_goal_state(configuration_name='close')
    plan_exec(robot, grip, '5 gripper close')
    log_joints(robot, 'gripper close')

def approach1_pose(ctx):
    """3 pose 목표(위치 + 방향). 4자유도라 방향을 마음대로 고를 수 없다."""
    robot, arm, ee = ctx['robot'], ctx['arm'], ctx['approach1_ee']
    # 3a: FK와 같은 위치 + 같은 방향 → 도달 가능 → 계획 성공
    ps = PoseStamped()
    ps.header.frame_id = FRAME
    ps.pose = ee
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=ps, pose_link=EE)
    plan_exec(robot, arm, '3 pose target', execute=True)
    log_joints(robot, 'approach1 pose')

def approach1_position(ctx):
    """4 위치만 목표 (반지름 5 mm 구). 손목 각은 IK가 정한다."""
    robot, arm, ee = ctx['robot'], ctx['arm'], ctx['approach1_ee']
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = EE
    pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
    pp = Pose()
    pp.position.x = ee.position.x
    pp.position.y = ee.position.y
    pp.position.z = ee.position.z
    pp.orientation.w = 1.0
    pc.constraint_region.primitive_poses.append(pp)
    pc.weight = 1.0
    c.position_constraints.append(pc)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(motion_plan_constraints=[c])
    plan_exec(robot, arm, '4 position-only')
    q = current(robot)
    LOG.info(f'  손목 각 j2+j3+j4 = {q[1] + q[2] + q[3]:+.3f} rad '
             f'(2단계 approach: {sum(APPROACH1[1:]):+.3f}) — 같은 위치여도 실행마다 달라진다')
    log_joints(robot, 'approach1 position')

def approach2_pose(ctx):
    """3 pose 목표(위치 + 방향). 4자유도라 방향을 마음대로 고를 수 없다."""
    robot, arm, ee = ctx['robot'], ctx['arm'], ctx['approach2_ee']
    # 3a: FK와 같은 위치 + 같은 방향 → 도달 가능 → 계획 성공
    ps = PoseStamped()
    ps.header.frame_id = FRAME
    ps.pose = ee
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=ps, pose_link=EE)
    plan_exec(robot, arm, '3 pose target', execute=True)
    log_joints(robot, 'approach2 pose')

def approach2_position(ctx):
    """4 위치만 목표 (반지름 5 mm 구). 손목 각은 IK가 정한다."""
    robot, arm, ee = ctx['robot'], ctx['arm'], ctx['approach2_ee']
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = EE
    pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
    pp = Pose()
    pp.position.x = ee.position.x
    pp.position.y = ee.position.y
    pp.position.z = ee.position.z
    pp.orientation.w = 1.0
    pc.constraint_region.primitive_poses.append(pp)
    pc.weight = 1.0
    c.position_constraints.append(pc)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(motion_plan_constraints=[c])
    plan_exec(robot, arm, '4 position-only')
    q = current(robot)
    LOG.info(f'  손목 각 j2+j3+j4 = {q[1] + q[2] + q[3]:+.3f} rad '
             f'(2단계 approach: {sum(APPROACH1[1:]):+.3f}) — 같은 위치여도 실행마다 달라진다')
    log_joints(robot, 'approach2 position')

def approach1_obstacle(ctx):
    """6 충돌 물체 회피: 가상 상자를 planning scene에 넣고 home → approach."""
    robot, arm, model = ctx['robot'], ctx['arm'], ctx['model']
    go_named(robot, arm, 'home', '6 start from home')

    # (1) 상자를 moveit_py planning scene에 추가 (RViz/move_group에는 보이지 않음)
    psm = robot.get_planning_scene_monitor()
    co = CollisionObject()
    co.header.frame_id = FRAME
    co.id = 'box'
    co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=BOX_SIZE))
    bp = Pose()
    bp.position.x, bp.position.y, bp.position.z = BOX_POS
    bp.orientation.w = 1.0
    co.primitive_poses.append(bp)
    co.operation = CollisionObject.ADD
    with psm.read_write() as scene:
        scene.apply_collision_object(co)
        scene.current_state.update()
    LOG.info(f'  상자 추가: 위치 {BOX_POS} m, 크기 {BOX_SIZE} m ({FRAME} 기준)')

    def colliding(qs):                             # 그리퍼 등 나머지 관절은 현재 값 그대로 두고 팔만 바꿔 검사
        with psm.read_only() as scene:
            s = copy.deepcopy(scene.current_state)
            s.set_joint_group_positions('arm', qs)
            s.update()
            return scene.is_state_colliding(robot_state=s, joint_model_group_name='arm')

    # (2) pick_demo 방식(관절 직선 보간)이면 몇 지점이 부딪히나
    hits = sum(colliding([h + (a - h) * k / 20 for h, a in zip(HOME, APPROACH1)]) for k in range(21))
    LOG.info(f'  관절 직선 경로 home→approach: 21개 중 {hits}개 지점 충돌 (pick_demo처럼 보내면 상자를 친다)')

    # (3) MoveIt 계획·실행 (최대 3회 재시도)
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=ctx['approach1_state'])
    r6 = plan_exec(robot, arm, '6 approach with box', attempts=3)

    # (4) MoveIt 경로 분석
    if r6:
        pts = r6.trajectory.get_robot_trajectory_msg().joint_trajectory.points
        zs = [joint_state(model, list(p.positions)[:4]).get_pose(EE).position.z for p in pts]
        bad = sum(colliding(list(p.positions)[:4]) for p in pts)
        LOG.info(f'  MoveIt 경로: 말단 z 최저 {min(zs):.3f} m (approach: {ctx["approach1_ee"].position.z:.3f}), '
                 f'충돌 지점 {bad}/{len(pts)} (0이어야 정상)')
    log_joints(robot, 'approach1 obstacle')

STEPS = {'1': home, '2': gripper_open, '3': ready1, '4': approach1_pose, '5': gripper_close, '6': lift, '7': ready2, '8': approach2_pose, '9': gripper_open, '10': home}

class ManipulatorControl(Node):

    def __init__(self, ctx):
        super().__init__('manipulator_control')
        self.ctx = ctx
        self.sub = self.create_subscription(
            String,
            '/manipulator_control',
            self.control_callback,
            10
        )

        self.pub = self.create_publisher(
            Bool,
            '/is_manipulator_completed',
            10
        )
        self.get_logger().info('Manipulator ready.')

    def control_callback(self, msg: String):
        if msg.data == 'pick_target':
            sequence = ['1', '2', '3', '4', '5', '6']
        elif msg.data == 'place_target':
            sequence = ['7', '8', '9', '10']
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')
            return
        
        result = Bool()

        try:
            for step in sequence:
                STEPS[step](self.ctx)
            result.data = True
        except Exception as e:
            self.get_logger().error(str(e))
            result.data = False
        self.pub.publish(result)

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='moveit_py 6단계 예제')
    ap.add_argument('--serve', action='store_true')
    ap.add_argument('--step', default='all', choices=list(STEPS) + ['all'], help='1~6 또는 all (기본 all)')
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    robot = MoveItPy(node_name='moveit_py_demo', config_dict=kit_moveit_config.build(use_sim_time=False))
    model = robot.get_robot_model()
    ctx = {
        'robot': robot,
        'model': model,
        'arm': robot.get_planning_component('arm'),
        'grip': robot.get_planning_component('gripper'),
    }
    # 2·3·4·6단계가 함께 쓰는 approach 상태와 그 말단 pose(FK)
    ctx['home_state'] = joint_state(model, HOME)
    ctx['ready1_state'] = joint_state(model, READY1)
    ctx['approach1_state'] = joint_state(model, APPROACH1)
    ctx['approach1_ee'] = ctx['approach1_state'].get_pose(EE)
    ctx['lift_state'] = joint_state(model, LIFT)
    ctx['ready2_state'] = joint_state(model, READY2)
    ctx['approach2_state'] = joint_state(model, APPROACH2)
    ctx['approach2_ee'] = ctx['approach2_state'].get_pose(EE)
    LOG.info(f'model frame = {model.model_frame}', f'pose 목표 frame = {FRAME}', f'속도·가속도 스케일 = 0.1')
    time.sleep(1.0)                                # /joint_states 수신 대기

    code = 0

    try:
        if args.serve:
            node = ManipulatorControl(ctx)

            try:
                rclpy.spin(node)
            finally:
                node.destroy_node()
        else:
            for key in(
                STEPS if args.step == 'all' 
                else [args.step]
            ):
                STEPS[key](ctx)
            LOG.info('모든 단계 완료')

    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        robot.shutdown()
        rclpy.try_shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)

if __name__ == '__main__':
    main()
