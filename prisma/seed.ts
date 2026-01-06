import { PrismaClient } from '@prisma/client'

const prisma = new PrismaClient()

async function main() {
  console.log('Starting seed...')

  // Create Learning Paths
  const beginnerPath = await prisma.learningPath.upsert({
    where: { id: 'path-beginner' },
    update: {},
    create: {
      id: 'path-beginner',
      title: 'Robotics Fundamentals',
      description: 'Master the fundamentals of robotics from scratch. Perfect for beginners starting their journey.',
      level: 'beginner',
      duration: '3 Months',
      order: 1,
    }
  })

  const intermediatePath = await prisma.learningPath.upsert({
    where: { id: 'path-intermediate' },
    update: {},
    create: {
      id: 'path-intermediate',
      title: 'Humanoid Development',
      description: 'Deep dive into humanoid robot control systems and development.',
      level: 'intermediate',
      duration: '6 Months',
      order: 2,
    }
  })

  const advancedPath = await prisma.learningPath.upsert({
    where: { id: 'path-advanced' },
    update: {},
    create: {
      id: 'path-advanced',
      title: 'AI & Robotics Mastery',
      description: 'Advanced AI algorithms and research-level robotics projects.',
      level: 'advanced',
      duration: '9 Months',
      order: 3,
    }
  })

  console.log('Created learning paths')

  // Create Courses for Beginner Path
  const electronicsCourse = await prisma.course.upsert({
    where: { slug: 'electronics-humanoid-robots' },
    update: {},
    create: {
      id: 'course-electronics',
      slug: 'electronics-humanoid-robots',
      title: 'Electronics of Humanoid Robots',
      description: 'Master core electronics components behind humanoid robots and learn how to build a complete robotic electronic system.',
      level: 'beginner',
      duration: '4 Weeks',
      thumbnail: '/images/poppy-robot.png',
      videoPreview: null,
      totalStudents: 5892,
      rating: 4.8,
      learningPathId: beginnerPath.id,
      order: 1,
    }
  })

  const pythonCourse = await prisma.course.upsert({
    where: { slug: 'python-robotics' },
    update: {},
    create: {
      id: 'course-python',
      slug: 'python-robotics',
      title: 'Python for Robotics',
      description: 'Learn Python programming specifically for robotics applications and simulations.',
      level: 'beginner',
      duration: '3 Weeks',
      thumbnail: '/images/ros2-diagram.png',
      videoPreview: null,
      totalStudents: 7234,
      rating: 4.9,
      learningPathId: beginnerPath.id,
      order: 2,
    }
  })

  // Create Courses for Intermediate Path
  const g1FundamentalsCourse = await prisma.course.upsert({
    where: { slug: 'unitree-g1-fundamentals' },
    update: {},
    create: {
      id: 'course-g1',
      slug: 'unitree-g1-fundamentals',
      title: 'Unitree G1 Fundamentals',
      description: 'Master the G1 humanoid robot: walking control, navigation, and perception systems.',
      level: 'intermediate',
      duration: '3 Days',
      thumbnail: '/images/unitree-g1.png',
      videoPreview: null,
      totalStudents: 2453,
      rating: 4.7,
      learningPathId: intermediatePath.id,
      order: 1,
    }
  })

  const controlSystemsCourse = await prisma.course.upsert({
    where: { slug: 'control-systems' },
    update: {},
    create: {
      id: 'course-control',
      slug: 'control-systems',
      title: 'Control Systems',
      description: 'Master control systems to achieve stable, adaptive, and efficient movements for humanoid robots.',
      level: 'intermediate',
      duration: '6 Weeks',
      thumbnail: '/images/simulation-interface.png',
      videoPreview: null,
      totalStudents: 1892,
      rating: 4.6,
      learningPathId: intermediatePath.id,
      order: 2,
    }
  })

  // Create Courses for Advanced Path
  const ros2AiCourse = await prisma.course.upsert({
    where: { slug: 'ros2-ai-integration' },
    update: {},
    create: {
      id: 'course-ros2-ai',
      slug: 'ros2-ai-integration',
      title: 'ROS2 & AI Integration',
      description: 'Advanced ROS2 programming with AI algorithms for humanoid robots.',
      level: 'advanced',
      duration: '6 Weeks',
      thumbnail: '/images/simulation-interface.png',
      videoPreview: null,
      totalStudents: 1245,
      rating: 4.9,
      learningPathId: advancedPath.id,
      order: 1,
    }
  })

  const aiAlgorithmsCourse = await prisma.course.upsert({
    where: { slug: 'ai-algorithms' },
    update: {},
    create: {
      id: 'course-ai',
      slug: 'ai-algorithms',
      title: 'AI Algorithm Systems',
      description: 'Master advanced AI algorithms to program humanoid robots with autonomous capabilities.',
      level: 'advanced',
      duration: '8 Weeks',
      thumbnail: '/images/simulation-interface.png',
      videoPreview: null,
      totalStudents: 987,
      rating: 4.8,
      learningPathId: advancedPath.id,
      order: 2,
    }
  })

  console.log('Created courses')

  // Create Modules for Electronics Course
  const module1 = await prisma.module.upsert({
    where: { id: 'module-electronics-1' },
    update: {
      title: 'Introduction to Electronics',
      description: 'Learn the basics of electronic components and circuits',
      order: 1,
      courseId: electronicsCourse.id,
    },
    create: {
      id: 'module-electronics-1',
      title: 'Introduction to Electronics',
      description: 'Learn the basics of electronic components and circuits',
      order: 1,
      courseId: electronicsCourse.id,
    }
  })

  const module2 = await prisma.module.upsert({
    where: { id: 'module-electronics-2' },
    update: {
      title: 'Actuators and Motors',
      description: 'Understand different types of actuators and motor control',
      order: 2,
      courseId: electronicsCourse.id,
    },
    create: {
      id: 'module-electronics-2',
      title: 'Actuators and Motors',
      description: 'Understand different types of actuators and motor control',
      order: 2,
      courseId: electronicsCourse.id,
    }
  })

  // Delete existing lessons for electronics course to avoid duplicates
  await prisma.lesson.deleteMany({
    where: {
      moduleId: {
        in: [module1.id, module2.id]
      }
    }
  })

  // Create Lessons
  await prisma.lesson.createMany({
    data: [
      {
        id: 'lesson-1-1',
        title: 'Electronic Components Overview',
        content: 'Introduction to resistors, capacitors, transistors, and other essential components.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 1800,
        order: 1,
        moduleId: module1.id,
        codeTemplate: null,
        language: null,
        simulationConfig: null,
      },
      {
        id: 'lesson-1-2',
        title: 'Building Your First Circuit',
        content: 'Hands-on tutorial for building a simple LED circuit.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ElephantsDream.mp4',
        videoDuration: 2400,
        order: 2,
        moduleId: module1.id,
        codeTemplate: null,
        language: null,
        simulationConfig: null,
      },
      {
        id: 'lesson-2-1',
        title: 'DC Motors Basics',
        content: 'Learn how DC motors work and how to control them.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ForBiggerBlazes.mp4',
        videoDuration: 2100,
        order: 1,
        moduleId: module2.id,
        codeTemplate: null,
        language: null,
        simulationConfig: null,
      },
    ]
  })

  console.log('Created modules and lessons')

  // Create Modules for Unitree G1 Fundamentals Course
  const g1Module1 = await prisma.module.upsert({
    where: { id: 'module-g1-1' },
    update: {
      title: 'Introduction to Unitree G1',
      description: 'Get familiar with the G1 humanoid robot platform, hardware specifications, and basic setup.',
      order: 1,
      courseId: g1FundamentalsCourse.id,
    },
    create: {
      id: 'module-g1-1',
      title: 'Introduction to Unitree G1',
      description: 'Get familiar with the G1 humanoid robot platform, hardware specifications, and basic setup.',
      order: 1,
      courseId: g1FundamentalsCourse.id,
    }
  })

  const g1Module2 = await prisma.module.upsert({
    where: { id: 'module-g1-2' },
    update: {
      title: 'Walking Control Systems',
      description: 'Master bipedal locomotion, balance control, and gait generation for the G1 robot.',
      order: 2,
      courseId: g1FundamentalsCourse.id,
    },
    create: {
      id: 'module-g1-2',
      title: 'Walking Control Systems',
      description: 'Master bipedal locomotion, balance control, and gait generation for the G1 robot.',
      order: 2,
      courseId: g1FundamentalsCourse.id,
    }
  })

  const g1Module3 = await prisma.module.upsert({
    where: { id: 'module-g1-3' },
    update: {
      title: 'Navigation and Path Planning',
      description: 'Learn how to program the G1 for autonomous navigation, obstacle avoidance, and path planning.',
      order: 3,
      courseId: g1FundamentalsCourse.id,
    },
    create: {
      id: 'module-g1-3',
      title: 'Navigation and Path Planning',
      description: 'Learn how to program the G1 for autonomous navigation, obstacle avoidance, and path planning.',
      order: 3,
      courseId: g1FundamentalsCourse.id,
    }
  })

  const g1Module4 = await prisma.module.upsert({
    where: { id: 'module-g1-4' },
    update: {
      title: 'Perception Systems',
      description: 'Understand vision systems, sensor fusion, and environmental perception for the G1 robot.',
      order: 4,
      courseId: g1FundamentalsCourse.id,
    },
    create: {
      id: 'module-g1-4',
      title: 'Perception Systems',
      description: 'Understand vision systems, sensor fusion, and environmental perception for the G1 robot.',
      order: 4,
      courseId: g1FundamentalsCourse.id,
    }
  })

  // Delete existing lessons for G1 course to avoid duplicates
  await prisma.lesson.deleteMany({
    where: {
      moduleId: {
        in: [g1Module1.id, g1Module2.id, g1Module3.id, g1Module4.id]
      }
    }
  })

  // Create Lessons for Unitree G1 Course
  await prisma.lesson.createMany({
    data: [
      // Module 1: Introduction to Unitree G1
      {
        id: 'lesson-g1-1-1',
        title: 'Unitree G1 Overview and Specifications',
        content: 'Introduction to the Unitree G1 humanoid robot. Learn about its hardware specifications, degrees of freedom, sensors, and capabilities. Understand the robot\'s physical structure, joint configurations, and computational architecture.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 1800, // 30 minutes
        order: 1,
        moduleId: g1Module1.id,
        codeTemplate: null,
        language: null,
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-1-2',
        title: 'G1 Development Environment Setup',
        content: 'Set up your development environment for G1 programming. Install required SDKs, configure ROS2 workspace, and establish communication with the robot. Learn about the G1 API and development tools.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ElephantsDream.mp4',
        videoDuration: 2400, // 40 minutes
        order: 2,
        moduleId: g1Module1.id,
        codeTemplate: `# Unitree G1 SDK Setup
import unitree_robot_sdk as urs

# Initialize G1 robot
robot = urs.Robot()
robot.connect()

# Check connection status
if robot.is_connected():
    print("G1 Robot Connected Successfully!")
    print(f"Battery Level: {robot.get_battery()}%")
else:
    print("Connection Failed")`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-1-3',
        title: 'Basic Robot Control and Safety',
        content: 'Learn fundamental robot control commands, safety protocols, and emergency stop procedures. Understand how to safely power on/off the robot and perform basic movements.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ForBiggerBlazes.mp4',
        videoDuration: 2100, // 35 minutes
        order: 3,
        moduleId: g1Module1.id,
        codeTemplate: `# Basic G1 Control
import unitree_robot_sdk as urs
import time

robot = urs.Robot()
robot.connect()

# Safety: Enable motors
robot.enable_motors()

# Basic pose control
robot.set_pose(stand=True)
time.sleep(2)

# Disable motors (safety)
robot.disable_motors()`,
        language: 'python',
        simulationConfig: null,
      },
      // Module 2: Walking Control Systems
      {
        id: 'lesson-g1-2-1',
        title: 'Bipedal Locomotion Fundamentals',
        content: 'Understand the principles of bipedal walking. Learn about center of mass, support polygons, and the physics of humanoid locomotion. Explore different walking gaits and their characteristics.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 2700, // 45 minutes
        order: 1,
        moduleId: g1Module2.id,
        codeTemplate: `# Walking Control Basics
import unitree_robot_sdk as urs
import numpy as np

robot = urs.Robot()
robot.connect()

# Configure walking parameters
walk_params = {
    'step_height': 0.05,  # meters
    'step_length': 0.15,  # meters
    'step_frequency': 1.0,  # Hz
    'body_height': 0.85   # meters
}

robot.set_walk_params(walk_params)`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-2-2',
        title: 'Balance Control and Stability',
        content: 'Master balance control algorithms for the G1. Learn about IMU feedback, center of pressure control, and adaptive balance strategies. Implement PID controllers for stable standing and walking.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 3000, // 50 minutes
        order: 2,
        moduleId: g1Module2.id,
        codeTemplate: `# Balance Control Implementation
import unitree_robot_sdk as urs
import numpy as np

class BalanceController:
    def __init__(self, kp=1.0, ki=0.1, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
    
    def update(self, current_angle, target_angle=0):
        error = target_angle - current_angle
        self.integral += error
        derivative = error - self.prev_error
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.prev_error = error
        return output

# Usage
controller = BalanceController()
robot = urs.Robot()
robot.connect()

while True:
    imu_data = robot.get_imu()
    correction = controller.update(imu_data.pitch)
    robot.adjust_balance(correction)`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-2-3',
        title: 'Gait Generation and Walking Patterns',
        content: 'Implement different walking patterns: forward, backward, turning, and side-stepping. Learn to generate smooth gait trajectories and adapt walking speed dynamically.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 2700, // 45 minutes
        order: 3,
        moduleId: g1Module2.id,
        codeTemplate: `# Gait Generation
import unitree_robot_sdk as urs
import numpy as np

def generate_gait(step_length, step_width, step_height, num_steps):
    """Generate walking gait trajectory"""
    trajectory = []
    
    for i in range(num_steps):
        # Left foot step
        left_foot = {
            'x': step_length * (i + 0.5),
            'y': step_width / 2,
            'z': step_height if i % 2 == 0 else 0
        }
        # Right foot step
        right_foot = {
            'x': step_length * (i + 1),
            'y': -step_width / 2,
            'z': step_height if i % 2 == 1 else 0
        }
        trajectory.append({'left': left_foot, 'right': right_foot})
    
    return trajectory

# Execute walking
robot = urs.Robot()
robot.connect()
gait = generate_gait(0.15, 0.12, 0.05, 10)
robot.execute_gait(gait)`,
        language: 'python',
        simulationConfig: null,
      },
      // Module 3: Navigation and Path Planning
      {
        id: 'lesson-g1-3-1',
        title: 'Path Planning Algorithms',
        content: 'Implement path planning algorithms for the G1 robot. Learn about A* pathfinding, RRT (Rapidly-exploring Random Tree), and dynamic obstacle avoidance. Create smooth navigation trajectories.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 3000, // 50 minutes
        order: 1,
        moduleId: g1Module3.id,
        codeTemplate: `# A* Path Planning for G1
import numpy as np
from queue import PriorityQueue

class AStarPlanner:
    def __init__(self, grid_size=20):
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size))
    
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def find_path(self, start, goal):
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while not frontier.empty():
            current = frontier.get()[1]
            
            if current == goal:
                break
            
            for next in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current
        
        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        return path[::-1]
    
    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                if self.grid[ny][nx] == 0:  # Not an obstacle
                    neighbors.append((nx, ny))
        return neighbors`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-3-2',
        title: 'Obstacle Avoidance and Dynamic Navigation',
        content: 'Implement real-time obstacle detection and avoidance. Learn to use LiDAR and depth sensors for dynamic navigation. Create reactive behaviors for unexpected obstacles.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 2700, // 45 minutes
        order: 2,
        moduleId: g1Module3.id,
        codeTemplate: `# Obstacle Avoidance
import unitree_robot_sdk as urs
import numpy as np

class ObstacleAvoidance:
    def __init__(self, safety_distance=0.5):
        self.safety_distance = safety_distance
    
    def detect_obstacles(self, lidar_data):
        """Process LiDAR data to detect obstacles"""
        obstacles = []
        for angle, distance in lidar_data:
            if distance < self.safety_distance:
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                obstacles.append((x, y))
        return obstacles
    
    def compute_avoidance_vector(self, obstacles, target_direction):
        """Compute direction to avoid obstacles"""
        if not obstacles:
            return target_direction
        
        # Calculate repulsion from obstacles
        avoidance = np.array([0.0, 0.0])
        for obs_x, obs_y in obstacles:
            direction = np.array([obs_x, obs_y])
            distance = np.linalg.norm(direction)
            if distance > 0:
                repulsion = -direction / (distance ** 2)
                avoidance += repulsion
        
        # Combine with target direction
        final_direction = target_direction + avoidance * 0.5
        return final_direction / np.linalg.norm(final_direction)

# Usage
robot = urs.Robot()
robot.connect()
avoidance = ObstacleAvoidance()

while True:
    lidar = robot.get_lidar()
    obstacles = avoidance.detect_obstacles(lidar)
    direction = avoidance.compute_avoidance_vector(obstacles, [1, 0])
    robot.walk(direction)`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-3-3',
        title: 'Localization and Mapping',
        content: 'Implement SLAM (Simultaneous Localization and Mapping) for the G1 robot. Learn about odometry, landmark detection, and map building. Create accurate robot localization systems.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 3000, // 50 minutes
        order: 3,
        moduleId: g1Module3.id,
        codeTemplate: `# Simple SLAM Implementation
import unitree_robot_sdk as urs
import numpy as np

class SimpleSLAM:
    def __init__(self):
        self.map = np.zeros((100, 100))  # 10m x 10m map
        self.robot_pose = np.array([50, 50, 0])  # x, y, theta
        self.landmarks = []
    
    def update_odometry(self, velocity, angular_velocity, dt):
        """Update robot position using odometry"""
        dx = velocity * np.cos(self.robot_pose[2]) * dt
        dy = velocity * np.sin(self.robot_pose[2]) * dt
        dtheta = angular_velocity * dt
        
        self.robot_pose[0] += dx
        self.robot_pose[1] += dy
        self.robot_pose[2] += dtheta
    
    def update_map(self, lidar_data):
        """Update map with LiDAR observations"""
        for angle, distance in lidar_data:
            # Convert to map coordinates
            map_x = int(self.robot_pose[0] + distance * np.cos(angle + self.robot_pose[2]))
            map_y = int(self.robot_pose[1] + distance * np.sin(angle + self.robot_pose[2]))
            
            if 0 <= map_x < 100 and 0 <= map_y < 100:
                self.map[map_y][map_x] = 1  # Mark as obstacle
    
    def get_robot_position(self):
        return self.robot_pose[:2]

# Usage
robot = urs.Robot()
robot.connect()
slam = SimpleSLAM()

while True:
    odom = robot.get_odometry()
    slam.update_odometry(odom.velocity, odom.angular_velocity, 0.1)
    
    lidar = robot.get_lidar()
    slam.update_map(lidar)
    
    position = slam.get_robot_position()
    print(f"Robot Position: ({position[0]:.2f}, {position[1]:.2f})")`,
        language: 'python',
        simulationConfig: null,
      },
      // Module 4: Perception Systems
      {
        id: 'lesson-g1-4-1',
        title: 'Vision Systems and Camera Integration',
        content: 'Integrate and use the G1\'s vision systems. Learn about camera calibration, image processing, and computer vision algorithms. Implement object detection and recognition.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 2700, // 45 minutes
        order: 1,
        moduleId: g1Module4.id,
        codeTemplate: `# Vision System Integration
import unitree_robot_sdk as urs
import cv2
import numpy as np

class VisionSystem:
    def __init__(self):
        self.camera = urs.Camera()
        self.camera.initialize()
    
    def capture_frame(self):
        """Capture image from G1 camera"""
        return self.camera.get_frame()
    
    def detect_objects(self, frame):
        """Simple object detection using color thresholding"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color range (example: red objects)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                objects.append({'x': x, 'y': y, 'width': w, 'height': h})
        
        return objects
    
    def get_object_position(self, object_bbox, camera_params):
        """Convert 2D image coordinates to 3D world coordinates"""
        # Simplified depth estimation
        depth = camera_params['depth']
        fx, fy = camera_params['focal_length']
        cx, cy = camera_params['center']
        
        x_img, y_img = object_bbox['x'], object_bbox['y']
        x_world = (x_img - cx) * depth / fx
        y_world = (y_img - cy) * depth / fy
        
        return (x_world, y_world, depth)

# Usage
robot = urs.Robot()
robot.connect()
vision = VisionSystem()

while True:
    frame = vision.capture_frame()
    objects = vision.detect_objects(frame)
    
    for obj in objects:
        print(f"Detected object at: {obj['x']}, {obj['y']}")`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-4-2',
        title: 'Sensor Fusion and Multi-Modal Perception',
        content: 'Combine data from multiple sensors (cameras, LiDAR, IMU) for robust perception. Learn sensor fusion techniques, Kalman filtering, and how to create a unified perception system.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 3000, // 50 minutes
        order: 2,
        moduleId: g1Module4.id,
        codeTemplate: `# Sensor Fusion Implementation
import unitree_robot_sdk as urs
import numpy as np
from scipy.linalg import inv

class SensorFusion:
    def __init__(self):
        self.state = np.array([0, 0, 0])  # x, y, theta
        self.covariance = np.eye(3) * 0.1
    
    def kalman_update(self, measurement, measurement_covariance, H):
        """Kalman filter update step"""
        # Prediction (simplified - no motion model here)
        P = self.covariance
        
        # Update
        K = P @ H.T @ inv(H @ P @ H.T + measurement_covariance)
        self.state = self.state + K @ (measurement - H @ self.state)
        self.covariance = (np.eye(3) - K @ H) @ P
    
    def fuse_sensors(self, camera_data, lidar_data, imu_data):
        """Fuse data from multiple sensors"""
        # Camera provides position estimate
        if camera_data:
            H_camera = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]])
            self.kalman_update(camera_data, np.eye(3) * 0.5, H_camera)
        
        # LiDAR provides distance measurements
        if lidar_data:
            # Process LiDAR to get position estimate
            lidar_pos = self.process_lidar(lidar_data)
            H_lidar = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]])
            self.kalman_update(lidar_pos, np.eye(3) * 0.3, H_lidar)
        
        # IMU provides orientation
        if imu_data:
            H_imu = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 1]])
            imu_measurement = np.array([0, 0, imu_data['yaw']])
            self.kalman_update(imu_measurement, np.eye(3) * 0.1, H_imu)
        
        return self.state
    
    def process_lidar(self, lidar_data):
        """Process LiDAR data to estimate position"""
        # Simplified: use average distance in front
        front_distances = [d for a, d in lidar_data if -0.5 < a < 0.5]
        if front_distances:
            avg_distance = np.mean(front_distances)
            return np.array([avg_distance, 0, 0])
        return np.array([0, 0, 0])

# Usage
robot = urs.Robot()
robot.connect()
fusion = SensorFusion()

while True:
    camera = robot.get_camera()
    lidar = robot.get_lidar()
    imu = robot.get_imu()
    
    state = fusion.fuse_sensors(camera, lidar, imu)
    print(f"Fused State: x={state[0]:.2f}, y={state[1]:.2f}, theta={state[2]:.2f}")`,
        language: 'python',
        simulationConfig: null,
      },
      {
        id: 'lesson-g1-4-3',
        title: 'Environmental Understanding and Scene Analysis',
        content: 'Implement advanced perception for understanding the environment. Learn about semantic segmentation, scene understanding, and context-aware navigation. Create intelligent behaviors based on environmental context.',
        videoUrl: 'https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4',
        videoDuration: 2700, // 45 minutes
        order: 3,
        moduleId: g1Module4.id,
        codeTemplate: `# Scene Analysis and Understanding
import unitree_robot_sdk as urs
import cv2
import numpy as np

class SceneAnalyzer:
    def __init__(self):
        self.camera = urs.Camera()
        self.object_classes = {
            'door': {'color': (100, 100, 100), 'action': 'approach'},
            'person': {'color': (0, 0, 255), 'action': 'follow'},
            'obstacle': {'color': (0, 255, 0), 'action': 'avoid'},
        }
    
    def analyze_scene(self, frame):
        """Analyze the scene and determine appropriate actions"""
        analysis = {
            'objects': [],
            'navigation_hints': [],
            'recommended_action': 'continue'
        }
        
        # Detect objects
        objects = self.detect_objects(frame)
        analysis['objects'] = objects
        
        # Determine navigation hints
        for obj in objects:
            if obj['class'] == 'door':
                analysis['navigation_hints'].append({
                    'type': 'waypoint',
                    'position': obj['position'],
                    'action': 'navigate_to'
                })
            elif obj['class'] == 'person':
                analysis['navigation_hints'].append({
                    'type': 'follow',
                    'target': obj['position'],
                    'action': 'follow_target'
                })
            elif obj['class'] == 'obstacle':
                analysis['navigation_hints'].append({
                    'type': 'avoid',
                    'position': obj['position'],
                    'action': 'avoid_obstacle'
                })
        
        # Determine recommended action
        if any(h['type'] == 'follow' for h in analysis['navigation_hints']):
            analysis['recommended_action'] = 'follow_person'
        elif any(h['type'] == 'waypoint' for h in analysis['navigation_hints']):
            analysis['recommended_action'] = 'navigate_to_door'
        elif any(h['type'] == 'avoid' for h in analysis['navigation_hints']):
            analysis['recommended_action'] = 'avoid_obstacles'
        
        return analysis
    
    def detect_objects(self, frame):
        """Detect and classify objects in the scene"""
        # Simplified object detection
        objects = []
        # In real implementation, use deep learning models
        # Here's a placeholder structure
        return objects

# Usage
robot = urs.Robot()
robot.connect()
analyzer = SceneAnalyzer()

while True:
    frame = robot.get_camera_frame()
    analysis = analyzer.analyze_scene(frame)
    
    if analysis['recommended_action'] == 'follow_person':
        robot.follow_target(analysis['navigation_hints'][0]['target'])
    elif analysis['recommended_action'] == 'navigate_to_door':
        robot.navigate_to(analysis['navigation_hints'][0]['position'])
    elif analysis['recommended_action'] == 'avoid_obstacles':
        robot.avoid_obstacles([h['position'] for h in analysis['navigation_hints']])`,
        language: 'python',
        simulationConfig: null,
      },
    ]
  })

  console.log('Created Unitree G1 course modules and lessons')

  console.log('Seed completed successfully!')
}

main()
  .catch((e) => {
    console.error(e)
    process.exit(1)
  })
  .finally(async () => {
    await prisma.$disconnect()
  })
