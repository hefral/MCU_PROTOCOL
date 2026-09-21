"""启动 mcu_bridge。

用法：
    ros2 launch mcu_protocol bridge.launch.py
    ros2 launch mcu_protocol bridge.launch.py device:=/dev/ttyUSB1
    ros2 launch mcu_protocol bridge.launch.py device:=/tmp/mcu_sim   # 接 proto_sim.py
    ros2 launch mcu_protocol bridge.launch.py release_device:=false  # 不自动清理占用者

默认 device=/dev/mcu，是本机 udev 为 CH340 建的固定软链（见 README「串口环境准备」）。
命令行传入的 device 会覆盖 config/bridge.yaml 里的值 —— 换设备号不必改文件。

默认 release_device=true：启动前先结束残留的 bridge/launch 进程。串口是用 TIOCEXCL
独占打开的，上一次 launch 没退干净（终端窗口被直接关掉，节点成了孤儿）时，新节点
只会拿到一句 EBUSY，而现象看起来和硬件坏了、没插设备一模一样。
"""

import os
import signal
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 只清理「我们自己留下的残骸」，且必须能精确认出才算数：
#   - 节点：/proc/<pid>/exe 的可执行文件名是 mcu_bridge
#   - 父进程：命令行里带 "bin/ros2 launch"（ros2 脚本的路径）
# 刻意**不**用 "ros2 launch" 做匹配 —— `bash -c "ros2 launch ..."` 这类外壳进程的
# 命令行里也会出现这串字，但它不是 ros2 本体，误杀它会牵连到用户自己的脚本。
_TERM_GRACE_S = 3.0


def _parent_of(pid):
    """读 /proc/<pid>/stat 取父 PID。comm 里可能有空格和 ')'，故从最后一个 ') ' 切。"""
    try:
        with open('/proc/%d/stat' % pid) as f:
            return int(f.read().rsplit(') ', 1)[1].split()[1])
    except (OSError, IndexError, ValueError):
        return 0


def _ancestors(pid):
    out = []
    cur = _parent_of(pid)
    while cur > 1:
        out.append(cur)
        cur = _parent_of(cur)
    return out


def _protected_pids():
    """本次 launch 自己及其全部祖先 + PID 1。绝不能杀 —— 否则会把正在启动的
    这次 launch 直接干掉，表现成「一运行 launch 窗口就没了」。"""
    return set(_ancestors(os.getpid())) | {os.getpid(), 1}


def _cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as f:
            return f.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
    except OSError:
        return ''  # 别人的进程；读不到就当不认识，不动它


def _exe(pid):
    try:
        return os.readlink('/proc/%d/exe' % pid)
    except OSError:
        return ''


def _is_stale_ours(pid):
    """是否是我们自己留下的 mcu_bridge / ros2 launch 进程。"""
    if os.path.basename(_exe(pid)) == 'mcu_bridge':
        return True
    return 'bin/ros2 launch' in _cmdline(pid)


def _holders(real_device):
    """返回当前持有 real_device 的 PID 集合。"""
    found = set()
    try:
        names = os.listdir('/proc')
    except OSError:
        return found
    for name in names:
        if not name.isdigit():
            continue
        pid = int(name)
        fd_dir = '/proc/%d/fd' % pid
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink('%s/%s' % (fd_dir, fd)) == real_device:
                    found.add(pid)
                    break
            except OSError:
                continue
    return found


def _release_device(context):
    """OpaqueFunction：在节点启动前清掉占用 device 的残留进程。"""
    flag = str(context.launch_configurations.get('release_device', 'true')).lower()
    if flag not in ('true', '1', 'yes', 'on'):
        return []

    device = context.launch_configurations.get('device', '/dev/mcu')
    real_device = os.path.realpath(device)
    protected = _protected_pids()

    # 占用者是 mcu_bridge 子进程；顺手把它的祖先里同样是我们自己的 launch 也带上，
    # 否则只杀子进程会留下一个空转的 ros2 launch。
    targets = set()
    for pid in _holders(real_device):
        if pid in protected:
            continue
        if not _is_stale_ours(pid):
            continue
        targets.add(pid)
        targets.update(a for a in _ancestors(pid) if a not in protected and _is_stale_ours(a))

    if not targets:
        return []

    print('[mcu_bridge.launch] %s 被残留进程占用，正在结束：%s'
          % (device, ', '.join('PID %d' % p for p in sorted(targets))))
    for pid in sorted(targets):
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError as exc:
            print('[mcu_bridge.launch] 结束 PID %d 失败: %s' % (pid, exc))

    deadline = time.monotonic() + _TERM_GRACE_S
    while time.monotonic() < deadline:
        if not (_holders(real_device) - protected):
            print('[mcu_bridge.launch] 串口已释放')
            return []
        time.sleep(0.1)

    print('[mcu_bridge.launch] 警告：%s 仍被占用（PID %s），未强杀。节点会自行重试，'
          '启动后请看它的报错确认占用者'
          % (device, ', '.join(str(p) for p in sorted(_holders(real_device) - protected))))
    return []


def generate_launch_description():
    device = LaunchConfiguration('device')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/mcu',
            description='串口设备路径。默认 /dev/mcu 是 udev 为 CH340 建的固定软链，插拔不变号',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('mcu_protocol'), 'config', 'bridge.yaml']
            ),
            description='参数文件路径',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别。调试链路问题时用 debug',
        ),
        DeclareLaunchArgument(
            'release_device',
            default_value='true',
            description=(
                '启动前结束残留的 mcu_bridge / ros2 launch 进程。串口是 TIOCEXCL 独占的，'
                '上次 launch 没退干净会让新节点只拿到 EBUSY。设为 false 只保留报错诊断'
            ),
        ),
        # 必须排在 Node 之前 —— launch 按声明顺序执行 action。
        OpaqueFunction(function=_release_device),
        Node(
            package='mcu_protocol',
            executable='mcu_bridge',
            name='mcu_bridge',
            # 节点名必须与 config/bridge.yaml 里的键一致，否则参数被静默忽略。
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'device': device}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
