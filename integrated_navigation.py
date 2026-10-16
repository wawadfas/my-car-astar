import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from math import pow, atan2, sqrt, sin, cos, pi
import heapq
import numpy as np

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        # 여기가 바로 당신들이 수정할 파라미터
        # [참고] /amcl_pose는 로봇이 약 0.25m 이동하거나 약 11도 회전해야 갱신됨 -> 아래 값들은 이 갱신 간격을 고려해서 정함

        self.lookahead_dist = 0.35 # 전방 주시 거리 : 멀리 보냐, 가까이 보냐에 따른 차이 발생함
        # 0.5 -> 0.35 : 0.5에서는 크게 돌다 경로에서 0.24m 벗어나 기둥에 충돌.
        #               최소 회전 반경이 0.25m -> 0.175m로 줄어 경로에 붙어서 감 (테스트 시 최대 이탈 0.18m)
        #               위치 갱신 간격(약 0.2~0.25m)보다 작으면 좌우로 흔들리므로 그보다 조금 크게 설정

        self.linear_vel = 0.15 # 이동 속도 : 느리냐, 빠르냐에 따른 차이 발생함
        # 0.2 -> 0.15 : 회전 속도 = linear_vel * 2*sin(alpha) / lookahead_dist 이고 최대 1.0으로 제한됨.
        #               0.2면 최대 1.14가 나와 1.0으로 잘려서 공식만큼 못 돌고, 0.15면 최대 0.86이라 잘리지 않음
        #               (느릴수록 사람 검출 지연 동안 덜 움직이는 장점도 있음)

        self.stop_tolerance = 0.2 # 도착 인정 거리 : 가까우면 멈추냐, 멀어도 멈추냐에 따른 차이 발생함
        # 0.15 -> 0.2 : 0.15면 도착 원 지름이 0.3m라, 위치가 0.2~0.25m 간격으로 들어올 때 원을 건너뛰고 지나칠 수 있음.
        #               지름 0.4m로 키워 갱신 간격보다 크게 함 (테스트 시 도착 오차 7~18cm)

        # 장애물에 부딪히지 않기 위한 안전거리 (x칸)
        self.safety_margin = 6 # 맵을 격자로 구분했을 때의 칸수
        # 4 -> 6 : 4칸(0.2m)은 로봇 몸통 반지름과 비슷해 경로에서 조금만 벗어나도 기둥에 닿음.
        #          6칸(0.3m)으로 경로를 장애물에서 더 떨어뜨림 (대신 장애물 0.3m 이내는 목표로 찍을 수 없음)

        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0

        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = [] # 계획된 경로
        self.path_index = 0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        # map_server와 amcl은 마지막 메시지를 보관해 두므로, 나중에 켜져도 받을 수 있게 TRANSIENT_LOCAL로 구독
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, latched_qos)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # person_detector가 특정 객체를 검출하면 True를 보냄 -> 그동안 비상정지
        self.emergency_stop = False
        self.sub_estop = self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)

        # 이 주기를 바꾸는 것도 주행 퍼포먼스에 영향을 줄 수 있음
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Let's Run!")

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def estop_callback(self, msg):
        if msg.data != self.emergency_stop:
            if msg.data:
                self.get_logger().warn("Emergency stop!")
            else:
                self.get_logger().info("Emergency stop released.")
        self.emergency_stop = msg.data

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    # goal_pose 토픽 발행 시
    # 1. 맵을 그리드 형태로 변환한 다음 최단 경로를 구한다.
    # 2. 구한 경로를 다시 원래의 형태로 변환한 다음 발행한다.
    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None: return

        goal_pose = [msg.pose.position.x, msg.pose.position.y]

        # x,y 좌표 값을 그리드 상에서의 행,열 로 변환하는 작업
        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)

        self.get_logger().info("Calculating Path...")
        path_grid = self.run_astar(start_grid, goal_grid)

        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
            self.get_logger().info("Path Found! Go!")
        else:
            self.get_logger().warn("No Path Found.")

    # 안전거리(마진)를 유지하는지 체크.... 안전거리보다 좁혀지지 않게!
    def check_safety(self, y, x) :
        margin = self.safety_margin

        for r in range(y - margin, y + margin + 1):
            for c in range(x - margin, x + margin + 1):
                if 0 <= r < self.map_height and 0 <= c < self.map_width:
                    if self.map_data[r][c] != 0: # 0이 아닌 게 장애물
                        return False
        return True

    def run_astar(self, start, end):
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0] < self.map_height and 0 <= end[1] < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []
        heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            for move in moves:
                ny, nx = current_node.position[0] + move[0], current_node.position[1] + move[1]
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] != 0: continue

                if not self.check_safety(ny, nx) : continue

                new_node = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + 1
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    # 경로는 에이스타가 구해줬다는 가정 하에, 나는 이동만 하면 돼!
    def control_loop(self):
        if not self.global_path: return

        # 비상정지 중에는 경로는 그대로 두고 멈춰 있기만 함 (해제되면 이어서 주행)
        if self.emergency_stop:
            self.stop_robot()
            return

        final_goal = self.global_path[-1] # 목적지

        # 목적지까지 가기 위해 계산해 본 거리
        dist_to_final = sqrt((final_goal[0]-self.current_pose[0])**2 + (final_goal[1]-self.current_pose[1])**2)

        if dist_to_final < self.stop_tolerance:
            self.global_path = []
            self.stop_robot()
            return

        # 도착 지점의 좌표
        target_x, target_y = final_goal

        # 경로가 점 10개짜리 경로라고 쳐봐....
        # 그러면 10번의 인덱싱을 할 수 있는거야!
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px - self.current_pose[0])**2 + (py - self.current_pose[1])**2)
            # 내가 주시하는 거리보다 조금 먼 정도의 목적지를 설정
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py # 적절한 타겟으로 갱신!
                self.path_index = i
                break

        # 새로 갱신한 현실적인 목적지까지의 거리 기반으로 계산
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        alpha = atan2(dy, dx) - self.current_yaw

        if alpha > pi: alpha -= 2*pi
        elif alpha < -pi: alpha += 2*pi

        angular_velocity = self.linear_vel * (2.0 * sin(alpha)) / self.lookahead_dist

        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = angular_velocity

        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0

        # 목표가 90도 넘게 틀어져 있으면(뒤쪽) 전진하지 않고 제자리에서 먼저 회전
        # (Pure Pursuit 공식은 목표가 뒤쪽일수록 sin(alpha)가 0에 가까워져 거의 돌지 않음)
        if abs(alpha) > pi / 2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if alpha > 0 else -0.5

        self.pub_cmd.publish(cmd)

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution), int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0], (grid[0]*self.map_resolution)+self.map_origin[1]]

    # 계획된 경로 발행
    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    # Ctrl+C 때 ROS가 먼저 꺼지지 않게 해서, 종료 직전에 정지 명령을 보낼 수 있게 함
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
