# rgbd_slam.launch.py — rgbd_odometry + rtabmap + slam_node
#
#   bag 재생 : ros2 launch rgbd_slam rgbd_slam.launch.py bag:=<bag폴더> [mount:=3]
#   실시간   : ros2 launch rgbd_slam rgbd_slam.launch.py [mount:=3]
#
#   bag:= 를 주면 bag 시계(use_sim_time)로 돌고, 재생이 끝나면 저절로 종료한다.
#   안 주면 실제 센서 토픽을 받고 Ctrl-C 로 종료한다. 둘 다 종료 시 slam_node 가 요약을 찍는다.
#   rtabmap 지도(db)는 매 실행 새로 만든다.
#
# 설정 파일 (config/)
#   param.yaml : 입력 토픽(전부), slam_node — 차체<->카메라 변환, 좌표계, 사전 측정, 노이즈, 루프 게이트
#   rtab.yaml  : rgbd_odometry·rtabmap — VO·루프 특징점(gftt|sift), 키프레임·루프 임계
#   mount:=    : 장착값(cam_rpy_deg, lever_arm)만 덮어쓰는 yaml. 경로 또는 config/mount_examples/ 의 이름(1, 2, 3)

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# rtabmap 고정 설정 — 건드리지 말 것
RTAB_ARGS = (
    "--delete_db_on_start "
    "--Mem/STMSize 5 "
    "--Mem/UseOdomFeatures false "               # VO 특징점이 루프용 사전에 섞이지 않게 (VO·루프 특징점을 따로 쓴다)
    "--RGBD/ForceOdom3DoF false "
    "--RGBD/ProximityBySpace false "
    "--RGBD/ProximityPathMaxNeighbors 15 "
    "--RGBD/LocalRadius 3.0 "
    "--RGBD/OptimizeMaxError 0.0 "
    "--RGBD/NeighborLinkRefining false "         # 켜면 이웃링크가 순수 VO 측정이 아니게 된다
    "--Rtabmap/CreateIntermediateNodes false"    # true 면 /info 가 없어 slam_node 가 링크를 노드에 못 붙인다
)

FEATURE = {'gftt': '8', 'sift': '1'}   # rtabmap Vis/FeatureType · Kp/DetectorStrategy 번호
VO_ODOM = "/vo/odom"                   # bag 안의 /odom 과 섞이지 않게 이름을 가른다
L = LaunchConfiguration
SHARE = get_package_share_directory('rgbd_slam')


def _is_bag(context):
    return bool(L('bag').perform(context))


def _rtab_cfg(context):
    path = os.path.expanduser(L('rtab').perform(context))
    with open(path) as f:
        c = yaml.safe_load(f)
    for k in ('vo_feature', 'loop_feature'):
        if c[k] not in FEATURE:
            raise RuntimeError('%s: %s 는 gftt | sift 중 하나 (지금 %s)' % (path, k, c[k]))
    return c


def _slam_params(param_path):
    with open(param_path) as f:
        return yaml.safe_load(f)['slam_node']['ros__parameters']


def _mount_path(context):
    """mount:= 가 파일 경로면 그대로, 아니면 config/mount_examples/<이름>.yaml"""
    m = L('mount').perform(context)
    if not m:
        return ''
    p = os.path.expanduser(m)
    if os.path.isfile(p):
        return p
    q = os.path.join(SHARE, 'config', 'mount_examples', m if m.endswith('.yaml') else m + '.yaml')
    if os.path.isfile(q):
        return q
    raise RuntimeError('mount 파일이 없다: %s (경로 또는 config/mount_examples/ 의 이름)' % m)


def _feature_args(kind, c):
    a = ['--Vis/FeatureType', FEATURE[kind]]
    if kind == 'sift':
        a += ['--SIFT/ContrastThreshold', str(c['sift_contrast']), '--SIFT/EdgeThreshold', str(c['sift_edge'])]
    return a


def _nodes(context, *a, **kw):
    bag = _is_bag(context)
    c = _rtab_cfg(context)
    param = os.path.expanduser(L('param').perform(context))
    p = _slam_params(param)
    rgb, depth, info = p['rgb_topic'], p['depth_topic'], p['camera_info_topic']

    sync = {'use_sim_time': bag, 'frame_id': 'camera_color_optical_frame',
            'approx_sync': True, 'wait_for_transform': 0.2,
            'qos_image': int(L('qos_image').perform(context)),
            'qos_camera_info': int(L('qos_image').perform(context)),
            'rgb_transport': L('rgb_transport').perform(context),
            'depth_transport': L('depth_transport').perform(context)}
    remap = [('rgb/image', rgb), ('depth/image', depth), ('rgb/camera_info', info), ('odom', VO_ODOM)]

    odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
        output=L('odom_output').perform(context),
        parameters=[dict(sync,
                         odom_frame_id='vo_odom',
                         publish_tf=True,                  # rtabmap 이 이 TF 를 찾는다
                         publish_null_when_lost=True)],    # 로스트 = 공분산 9999 (slam_node 가 감지)
        remappings=remap,
        arguments=['--Odom/ResetCountdown', '10'] + _feature_args(c['vo_feature'], c))

    rtab = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        # odom_frame_id 는 주지 말 것 — 주면 오도메트리를 TF 에서 읽다가 프레임을 버린다
        parameters=[dict(sync, publish_tf=False, subscribe_depth=True,
                         database_path=os.path.expanduser(L('db').perform(context)))],
        remappings=remap,
        arguments=RTAB_ARGS.split() + _feature_args(c['loop_feature'], c) + [
            '--Kp/DetectorStrategy', FEATURE[c['loop_feature']],
            '--Rtabmap/DetectionRate', str(c['detection_rate']),
            '--Rtabmap/LoopThr', str(c['loop_thr']),
            '--RGBD/LinearUpdate', str(c['linear_update']),
            '--RGBD/AngularUpdate', str(c['angular_update'])])

    params = [param]
    mount = _mount_path(context)
    if mount:
        params.append(mount)                       # 뒤 파일이 앞 파일을 덮어쓴다
    params.append({'use_sim_time': bag, 'vo_odom_topic': VO_ODOM})
    print('[rgbd_slam] %s  param=%s  mount=%s  rtab=%s (VO %s / 루프 %s)'
          % ('bag ' + L('bag').perform(context) if bag else '실시간',
             param, mount or '-', L('rtab').perform(context), c['vo_feature'], c['loop_feature']))
    slam = Node(package='rgbd_slam', executable='slam_node', name='slam_node', output='screen',
                parameters=params)

    nodes = [odom, rtab, slam]
    if L('rviz').perform(context).lower() == 'true':
        # 출력 포즈는 map 프레임인데 TF 트리에 map 이 없으면 rviz 가 못 그리므로,
        # map 을 띄우는 정적 TF 를 하나 둔다 (기하 의미 없음 — rviz 표시용)
        nodes += [
            Node(package='tf2_ros', executable='static_transform_publisher', name='map_tf_for_rviz',
                 arguments=['--frame-id', 'map', '--child-frame-id', 'rgbd_slam_rviz'],
                 parameters=[{'use_sim_time': bag}]),
            Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                 arguments=['-d', os.path.join(SHARE, 'config', 'rgbd_slam.rviz')],
                 parameters=[{'use_sim_time': bag}])]
    if L('viz').perform(context).lower() == 'true':
        nodes.append(Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
            parameters=[dict(sync, subscribe_depth=True, subscribe_odom_info=True)],
            remappings=remap + [('odom_info', '/odom_info')]))
    return nodes


def _prepare(context, *a, **kw):
    """실행 전 준비: 지난 db 지우기, bag 이면 폴더·토픽 확인과 로컬 전용 통신."""
    # 지난 실행 db 가 남아 있으면 rtabmap 이 옛 지도를 이어받아 링크가 엉뚱한 노드에 붙는다
    db = os.path.expanduser(L('db').perform(context))
    for f in (db, db + '-journal', db + '-wal', db + '-shm'):
        if os.path.exists(f):
            os.remove(f)
    if not _is_bag(context):
        return []

    bag = os.path.expanduser(L('bag').perform(context))
    meta = os.path.join(bag, 'metadata.yaml')
    if not os.path.isfile(meta):
        raise RuntimeError('ros2 bag 폴더가 아니다 (metadata.yaml 없음): %s' % bag)
    with open(meta) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']['topics_with_message_count']
    have = {t['topic_metadata']['name'] for t in info}
    p = _slam_params(os.path.expanduser(L('param').perform(context)))
    for k in ('imu_topic', 'wheel_odom_topic', 'rgb_topic', 'depth_topic', 'camera_info_topic'):
        if p[k] not in have:
            print('[rgbd_slam] ※ bag 에 %s (%s) 가 없다 — param.yaml 토픽 이름 확인' % (p[k], k))
    # bag 재생은 전부 로컬 — 같은 네트워크의 실제 센서 토픽이 재생분과 섞이지 않게 한다
    return [SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')]


def _play(context, *a, **kw):
    """bag 재생. 끝나면 8 s 뒤 전체 종료 (slam_node 가 종료하면서 요약을 찍는다)."""
    if not _is_bag(context):
        return []
    bag = os.path.expanduser(L('bag').perform(context))
    proc = ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock', '-r', L('rate').perform(context),
                               '--disable-keyboard-controls'], output='screen')
    stop = RegisterEventHandler(OnProcessExit(target_action=proc, on_exit=[
        TimerAction(period=8.0, actions=[EmitEvent(event=Shutdown(reason='bag 재생 완료'))])]))
    return [proc, stop]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bag', default_value='', description='ros2 bag 폴더 (비우면 실시간)'),
        DeclareLaunchArgument('param', default_value=os.path.join(SHARE, 'config', 'param.yaml')),
        DeclareLaunchArgument('rtab', default_value=os.path.join(SHARE, 'config', 'rtab.yaml')),
        DeclareLaunchArgument('mount', default_value=''),             # 장착값 yaml 경로 또는 예시 이름(1|2|3)
        DeclareLaunchArgument('db', default_value='/tmp/rgbd_slam.db'),
        DeclareLaunchArgument('rviz', default_value='false'),         # rviz 로 Fused 궤적 보기
        DeclareLaunchArgument('viz', default_value='false'),          # rtabmap_viz (특징점 매칭)
        DeclareLaunchArgument('odom_output', default_value='own_log'),  # rgbd_odometry 로그: own_log = 파일로만 | screen
        # bag 재생
        DeclareLaunchArgument('rate', default_value='1.0'),           # 재생 속도
        DeclareLaunchArgument('delay', default_value='5.0'),          # 노드 기동 후 재생까지 [s]
        # 영상 수신 (주로 실시간용): QoS 0=기본 1=reliable 2=best_effort,
        #   transport raw | compressed (WiFi 로 받을 때) / depth 는 raw | compressedDepth
        DeclareLaunchArgument('qos_image', default_value='0'),
        DeclareLaunchArgument('rgb_transport', default_value='raw'),
        DeclareLaunchArgument('depth_transport', default_value='raw'),
    ]
    return LaunchDescription(
        args + [OpaqueFunction(function=_prepare), OpaqueFunction(function=_nodes),
                TimerAction(period=L('delay'), actions=[OpaqueFunction(function=_play)])])
