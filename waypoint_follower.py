import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import yaml
import cv2
import os
import time
import math


class WaypointExecutor(Node):

    def __init__(self):
        super().__init__('waypoint_executor')
        self.bridge = CvBridge()
        self.image = None
        self.pose = None
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

    def image_callback(self, msg):
        self.image = msg

    def pose_callback(self, msg):
        self.pose = msg

    def load_pose_from_yaml(self, folder, filename):
        path = os.path.join(os.path.dirname(__file__), folder, filename)
        if not os.path.exists(path):
            return None
        with open(path, 'r') as f:
            return yaml.safe_load(f)['pose']

    def send_goal(self, pose_dict, label=''):

        print(f'➡️ {label} 이동 시작')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose_dict['x']
        goal.pose.pose.position.y = pose_dict['y']
        goal.pose.pose.position.z = 0.0
        theta = pose_dict['theta']
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        client.wait_for_server()
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f'❌ {label} 이동 실패')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        print(f'✅ {label} 도착 완료')
        return True

    def save_image_and_pose(self, index, capture_delay=4.0):
  
        time.sleep(capture_delay)

        wait_start = time.time()
        while self.image is None and time.time() - wait_start < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        base_path = os.path.join(os.path.dirname(__file__), 'logs', 'images', 'current')
        os.makedirs(base_path, exist_ok=True)


        try:
            img = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
            cv2.imwrite(os.path.join(base_path, f'{index}.jpg'), img)
        except Exception as e:
            print(f'⚠️ {index}.jpg 저장 실패: {e}')

        # 4) 포즈 저장

        pose_msg = self.pose.pose.pose
        q = pose_msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        pose_data = {

            'pose': {
                'x': pose_msg.position.x,
                'y': pose_msg.position.y,
                'theta': theta
            }

        }

        yaml_path = os.path.join(base_path, f'{index}.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(pose_data, f)
        print(f'💾 {index}.jpg / {index}.yaml 저장 완료')

def main():

    rclpy.init()
    node = WaypointExecutor()
    index = 1
    POSITION_THRESHOLD = 0.25  # 허용 오차: 0.25m

    while True:
        pose = node.load_pose_from_yaml('config', f'waypoint{index}.yaml')
        if pose is None:
            break

        # 1) 웨이포인트 이동

        node.send_goal(pose, label=f'웨이포인트 {index}')

        # 2) 현재 위치와 목표 위치 오차 계산

        current = node.pose.pose.pose.position
        dx = abs(current.x - pose['x'])
        dy = abs(current.y - pose['y'])



        # 3) 오차 메시지 출력 및 재시도 로직 추가

        if dx < POSITION_THRESHOLD and dy < POSITION_THRESHOLD:
            print(f'✅ {index} 위치 도달 (오차: {dx:.2f}, {dy:.2f}) — 정상 캡처')

        else:
            print(f'⚠️ {index} 위치 오차 too large ({dx:.2f}, {dy:.2f}) — 재시도')
            node.send_goal(pose, label=f'웨이포인트 {index} 재시도')
            current = node.pose.pose.pose.position
            dx2 = abs(current.x - pose['x'])
            dy2 = abs(current.y - pose['y'])
            if dx2 < POSITION_THRESHOLD and dy2 < POSITION_THRESHOLD:
                print(f'✅ {index} 재시도 성공 (오차: {dx2:.2f}, {dy2:.2f})')
            else:
                print(f'⚠️ {index} 재시도 실패 (오차: {dx2:.2f}, {dy2:.2f}) — 경고 후 캡처')


        time.sleep(2.0)
        node.save_image_and_pose(index, capture_delay=4.0)
        time.sleep(2.0)
        index += 1

    base_pose = node.load_pose_from_yaml('return_base', 'base.yaml')
    if base_pose:
        while not node.send_goal(base_pose, label='복귀'):
            print('🔁 복귀 재시도 중...')
        print('🏁 복귀 완료')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
