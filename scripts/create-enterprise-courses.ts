/**
 * Enterprise-Level Course Creation Script
 * Creates SOTA production-ready courses for NAVA-ROBOTICLEARN platform
 * Enhanced with NeoVerse, AvatarForcing, and VEO3
 */

import { PrismaClient } from '@prisma/client'

const prisma = new PrismaClient()

// Enterprise-level course structure for NAVA-ROBOTICLEARN platform
const enterpriseCourses = [
  {
    id: 'ros-fundamentals-enterprise',
    slug: 'ros-fundamentals-enterprise',
    title: 'ROS Fundamentals - Enterprise Edition',
    description: 'Master ROS (Robot Operating System) from zero to production-ready. Comprehensive enterprise-level course with hands-on simulations, real-world projects, and industry best practices.',
    level: 'beginner',
    duration: '8 Weeks',
    thumbnail: '/images/ros2-diagram.png',
    modules: [
      {
        title: 'Introduction to ROS',
        description: 'Understanding ROS architecture, core concepts, and ecosystem',
        lessons: [
          {
            title: 'What is ROS and Why It Matters',
            content: `# What is ROS and Why It Matters

ROS (Robot Operating System) is the industry standard for robotics development. This lesson introduces you to:

## Key Concepts
- ROS architecture and design principles
- Publisher-subscriber communication model
- Service and action patterns
- ROS package structure
- Workspace organization

## Industry Applications
- Autonomous vehicles
- Industrial automation
- Service robotics
- Research and development

## Learning Objectives
By the end of this lesson, you will:
- Understand ROS ecosystem and tools
- Know when to use ROS in projects
- Set up your development environment
- Create your first ROS package`,
            order: 1
          },
          {
            title: 'ROS Workspace Setup and Best Practices',
            content: `# ROS Workspace Setup and Best Practices

Learn enterprise-grade workspace organization and development workflows.

## Topics Covered
- Catkin workspace structure
- Package creation and management
- Build system (catkin_make vs catkin build)
- Environment setup
- Version control integration

## Hands-On Exercises
- Create a ROS workspace
- Build and source packages
- Configure IDE integration
- Set up development environment`,
            order: 2
          },
          {
            title: 'ROS Nodes, Topics, and Messages',
            content: `# ROS Nodes, Topics, and Messages

Master the fundamental communication patterns in ROS.

## Core Concepts
- Nodes: Independent processes
- Topics: Asynchronous communication
- Messages: Data structures
- Publishers and Subscribers

## Practical Examples
- Create publisher nodes
- Create subscriber nodes
- Custom message definitions
- Message inspection tools`,
            order: 3
          }
        ]
      },
      {
        title: 'ROS Communication Patterns',
        description: 'Deep dive into ROS communication mechanisms',
        lessons: [
          {
            title: 'Services and Actions',
            content: `# Services and Actions

Learn synchronous and asynchronous request-response patterns.

## Services
- Service definition
- Service clients and servers
- When to use services vs topics

## Actions
- Action definition
- Action clients and servers
- Progress feedback
- Cancellation handling

## Best Practices
- Choosing the right pattern
- Error handling
- Timeout management`,
            order: 1
          },
          {
            title: 'Parameter Server and Dynamic Reconfiguration',
            content: `# Parameter Server and Dynamic Reconfiguration

Manage configuration and runtime parameters effectively.

## Parameter Server
- Setting and getting parameters
- Parameter types
- Namespace organization

## Dynamic Reconfiguration
- Runtime parameter changes
- GUI tools
- Best practices`,
            order: 2
          }
        ]
      },
      {
        title: 'ROS Tools and Debugging',
        description: 'Master ROS development tools and debugging techniques',
        lessons: [
          {
            title: 'rviz, rqt, and Visualization Tools',
            content: `# ROS Visualization Tools

Professional visualization and debugging tools.

## rviz
- 3D visualization
- TF tree visualization
- Sensor data display
- Interactive markers

## rqt
- GUI tools suite
- Topic monitoring
- Service calling
- Parameter editing

## Best Practices
- Effective debugging workflows
- Performance monitoring`,
            order: 1
          },
          {
            title: 'rosbag and Data Recording',
            content: `# rosbag and Data Recording

Record and replay robot data for testing and analysis.

## rosbag
- Recording topics
- Filtering and compression
- Playback and analysis
- Best practices

## Use Cases
- Testing without hardware
- Data analysis
- Debugging
- Documentation`,
            order: 2
          }
        ]
      },
      {
        title: 'Advanced ROS Concepts',
        description: 'Enterprise-level ROS patterns and practices',
        lessons: [
          {
            title: 'TF (Transform) Library',
            content: `# TF (Transform) Library

Coordinate frame management in ROS.

## Concepts
- Transform trees
- Frame relationships
- Static and dynamic transforms
- TF broadcaster and listener

## Applications
- Multi-sensor fusion
- Robot kinematics
- Navigation stacks`,
            order: 1
          },
          {
            title: 'ROS Launch Files',
            content: `# ROS Launch Files

Orchestrate complex robot systems.

## Launch File Syntax
- Node launching
- Parameter passing
- Remapping
- Conditional execution

## Best Practices
- Modular launch files
- Parameter organization
- Error handling`,
            order: 2
          }
        ]
      }
    ]
  },
  {
    id: 'ros-navigation-enterprise',
    slug: 'ros-navigation-enterprise',
    title: 'ROS Navigation Stack - Enterprise Edition',
    description: 'Build production-ready navigation systems using ROS Navigation Stack. Learn SLAM, path planning, localization, and obstacle avoidance.',
    level: 'intermediate',
    duration: '10 Weeks',
    thumbnail: '/images/simulation-interface.png',
    modules: [
      {
        title: 'SLAM Fundamentals',
        description: 'Simultaneous Localization and Mapping',
        lessons: [
          {
            title: 'Introduction to SLAM',
            content: `# Introduction to SLAM

Simultaneous Localization and Mapping (SLAM) is fundamental to autonomous navigation.

## Key Concepts
- Localization vs Mapping
- SLAM problem formulation
- Sensor fusion
- Loop closure detection

## SLAM Algorithms
- EKF SLAM
- Particle Filter SLAM
- Graph-based SLAM
- Visual SLAM

## ROS Implementation
- gmapping
- cartographer
- ORB-SLAM integration`,
            order: 1
          },
          {
            title: 'Laser SLAM with gmapping',
            content: `# Laser SLAM with gmapping

Implement SLAM using LiDAR sensors.

## Setup
- Sensor configuration
- TF tree setup
- Parameter tuning

## Implementation
- Launch file configuration
- Map generation
- Quality assessment

## Best Practices
- Parameter optimization
- Map quality metrics`,
            order: 2
          }
        ]
      },
      {
        title: 'Path Planning and Obstacle Avoidance',
        description: 'Navigate robots safely through environments',
        lessons: [
          {
            title: 'Global Path Planning',
            content: `# Global Path Planning

Plan optimal paths from start to goal.

## Algorithms
- A* algorithm
- Dijkstra's algorithm
- RRT (Rapidly-exploring Random Tree)
- PRM (Probabilistic Roadmap)

## ROS Implementation
- navfn planner
- global_planner
- Custom planners

## Optimization
- Cost functions
- Heuristic design
- Performance tuning`,
            order: 1
          },
          {
            title: 'Local Path Planning and Obstacle Avoidance',
            content: `# Local Path Planning

Dynamic obstacle avoidance and local navigation.

## Algorithms
- DWA (Dynamic Window Approach)
- TEB (Timed Elastic Band)
- MPC (Model Predictive Control)

## ROS Implementation
- base_local_planner
- dwa_local_planner
- teb_local_planner

## Tuning
- Parameter optimization
- Safety margins
- Performance vs safety trade-offs`,
            order: 2
          }
        ]
      },
      {
        title: 'Localization and AMCL',
        description: 'Robot localization in known environments',
        lessons: [
          {
            title: 'AMCL (Adaptive Monte Carlo Localization)',
            content: `# AMCL Localization

Particle filter-based localization.

## Concepts
- Particle filters
- Sensor models
- Motion models
- Resampling

## Configuration
- Parameter tuning
- Sensor integration
- Performance optimization

## Best Practices
- Initial pose estimation
- Sensor fusion
- Failure recovery`,
            order: 1
          }
        ]
      }
    ]
  },
  {
    id: 'ros-perception-enterprise',
    slug: 'ros-perception-enterprise',
    title: 'ROS Perception and Computer Vision - Enterprise Edition',
    description: 'Advanced computer vision and perception systems for robotics. Image processing, object detection, depth estimation, and sensor fusion.',
    level: 'advanced',
    duration: '12 Weeks',
    thumbnail: '/images/learning-dashboard.png',
    modules: [
      {
        title: 'ROS Image Processing',
        description: 'Image acquisition and processing in ROS',
        lessons: [
          {
            title: 'Camera Integration and Calibration',
            content: `# Camera Integration and Calibration

Professional camera setup for robotics.

## Camera Drivers
- USB camera drivers
- Camera calibration
- Image rectification
- Synchronization

## ROS Packages
- usb_cam
- image_transport
- camera_calibration

## Best Practices
- Calibration procedures
- Quality assessment
- Multi-camera setups`,
            order: 1
          },
          {
            title: 'Image Processing with OpenCV',
            content: `# Image Processing with OpenCV

Advanced image processing in ROS.

## Topics
- Image filtering
- Feature detection
- Edge detection
- Morphological operations

## ROS Integration
- cv_bridge
- Image message conversion
- Real-time processing

## Applications
- Object detection
- Visual servoing
- Quality inspection`,
            order: 2
          }
        ]
      },
      {
        title: 'Object Detection and Recognition',
        description: 'Detect and recognize objects in real-time',
        lessons: [
          {
            title: 'Deep Learning for Object Detection',
            content: `# Deep Learning Object Detection

State-of-the-art object detection in ROS.

## Models
- YOLO integration
- TensorFlow ROS
- PyTorch integration
- Custom models

## ROS Implementation
- darknet_ros
- tensorflow_object_detector
- Performance optimization

## Best Practices
- Model selection
- Real-time constraints
- Hardware acceleration`,
            order: 1
          }
        ]
      }
    ]
  },
  {
    id: 'ros-manipulation-enterprise',
    slug: 'ros-manipulation-enterprise',
    title: 'ROS Manipulation - Enterprise Edition',
    description: 'Control robotic arms and manipulators. Inverse kinematics, motion planning, grasp planning, and force control.',
    level: 'advanced',
    duration: '10 Weeks',
    thumbnail: '/images/poppy-robot.png',
    modules: [
      {
        title: 'Robot Arm Fundamentals',
        description: 'Understanding robotic manipulators',
        lessons: [
          {
            title: 'Kinematics and Dynamics',
            content: `# Robot Arm Kinematics and Dynamics

Mathematical foundations of robotic manipulation.

## Forward Kinematics
- Denavit-Hartenberg parameters
- Transformation matrices
- Joint space to Cartesian space

## Inverse Kinematics
- Analytical solutions
- Numerical methods
- IK solvers

## Dynamics
- Lagrangian formulation
- Torque calculations
- Dynamic simulation

## ROS Implementation
- KDL (Kinematics and Dynamics Library)
- MoveIt! integration`,
            order: 1
          },
          {
            title: 'MoveIt! Framework',
            content: `# MoveIt! Framework

Industry-standard manipulation framework.

## Components
- Motion planning
- Collision checking
- Trajectory execution
- Grasp planning

## Setup
- URDF/SRDF configuration
- Planning scene setup
- Controller integration

## Best Practices
- Workspace definition
- Collision objects
- Performance optimization`,
            order: 2
          }
        ]
      },
      {
        title: 'Motion Planning',
        description: 'Plan safe and efficient robot motions',
        lessons: [
          {
            title: 'OMPL and Motion Planning',
            content: `# OMPL Motion Planning

Open Motion Planning Library integration.

## Planners
- RRT variants
- PRM
- CHOMP
- STOMP

## Configuration
- Planner selection
- Parameter tuning
- Performance optimization

## Applications
- Pick and place
- Assembly tasks
- Collaborative manipulation`,
            order: 1
          }
        ]
      }
    ]
  },
  {
    id: 'ros-ai-agents-enterprise',
    slug: 'ros-ai-agents-enterprise',
    title: 'AI Agents for Robotics - Enterprise Edition',
    description: 'Build intelligent robotic agents using AI and machine learning. Reinforcement learning, behavior trees, and autonomous decision-making.',
    level: 'advanced',
    duration: '14 Weeks',
    thumbnail: '/images/hero-robot.png',
    modules: [
      {
        title: 'Reinforcement Learning for Robotics',
        description: 'Train robots to learn from experience',
        lessons: [
          {
            title: 'RL Fundamentals for Robotics',
            content: `# Reinforcement Learning for Robotics

Train robots to learn optimal behaviors.

## Concepts
- MDP formulation
- Policy learning
- Value functions
- Exploration vs exploitation

## Algorithms
- Q-learning
- Policy gradients
- Actor-critic methods
- PPO, SAC, TD3

## ROS Integration
- Gym integration
- Simulation environments
- Real robot deployment

## Best Practices
- Reward design
- Safety constraints
- Transfer learning`,
            order: 1
          },
          {
            title: 'Behavior Trees for Robotics',
            content: `# Behavior Trees

Modular behavior design for robots.

## Concepts
- Tree structure
- Composite nodes
- Decorator nodes
- Leaf nodes

## ROS Implementation
- BehaviorTree.CPP
- Groot visualization
- Integration patterns

## Applications
- Task planning
- Reactive behaviors
- Failure recovery`,
            order: 2
          }
        ]
      }
    ]
  },
  {
    id: 'ros-fleet-management-enterprise',
    slug: 'ros-fleet-management-enterprise',
    title: 'Robot Fleet Management - Enterprise Edition',
    description: 'Manage multiple robots in production environments. Fleet coordination, task allocation, and distributed systems.',
    level: 'advanced',
    duration: '8 Weeks',
    thumbnail: '/images/unitree-g1.png',
    modules: [
      {
        title: 'Multi-Robot Systems',
        description: 'Coordinate multiple robots',
        lessons: [
          {
            title: 'Fleet Architecture',
            content: `# Robot Fleet Architecture

Design scalable multi-robot systems.

## Architecture Patterns
- Centralized coordination
- Distributed systems
- Hybrid approaches

## Communication
- ROS 2 DDS
- Message passing
- Service discovery

## Task Allocation
- Auction-based
- Market-based
- Centralized planning

## ROS Implementation
- Fleet management frameworks
- Task coordination
- Resource sharing`,
            order: 1
          }
        ]
      }
    ]
  }
]

async function createEnterpriseCourses() {
  console.log('🚀 Creating Enterprise-Level Courses...\n')
  console.log('='.repeat(60))

  // Create or get learning path
  const learningPath = await prisma.learningPath.upsert({
    where: { id: 'enterprise-ros-path' },
    update: {},
    create: {
      id: 'enterprise-ros-path',
      title: 'Enterprise ROS Mastery Path',
      description: 'Complete enterprise-level curriculum for ROS and robotics. From fundamentals to advanced topics, production-ready skills.',
      level: 'beginner',
      duration: '12 Months',
      order: 1
    }
  })

  console.log(`✅ Learning Path: ${learningPath.title}\n`)

  for (const courseData of enterpriseCourses) {
    console.log(`📚 Creating course: ${courseData.title}`)

    // Create course
    const course = await prisma.course.upsert({
      where: { slug: courseData.slug },
      update: {
        title: courseData.title,
        description: courseData.description,
        level: courseData.level,
        duration: courseData.duration,
        thumbnail: courseData.thumbnail,
        learningPathId: learningPath.id
      },
      create: {
        id: courseData.id,
        slug: courseData.slug,
        title: courseData.title,
        description: courseData.description,
        level: courseData.level,
        duration: courseData.duration,
        thumbnail: courseData.thumbnail,
        learningPathId: learningPath.id,
        order: enterpriseCourses.indexOf(courseData) + 1
      }
    })

    console.log(`   ✅ Course created: ${course.title}`)

    // Create modules and lessons
    for (let moduleIndex = 0; moduleIndex < courseData.modules.length; moduleIndex++) {
      const moduleData = courseData.modules[moduleIndex]
      
      const module = await prisma.module.upsert({
        where: { id: `${course.id}-module-${moduleIndex + 1}` },
        update: {
          title: moduleData.title,
          description: moduleData.description,
          order: moduleIndex + 1
        },
        create: {
          id: `${course.id}-module-${moduleIndex + 1}`,
          title: moduleData.title,
          description: moduleData.description,
          order: moduleIndex + 1,
          courseId: course.id
        }
      })

      console.log(`   📦 Module: ${module.title}`)

      // Create lessons
      for (const lessonData of moduleData.lessons) {
        const lesson = await prisma.lesson.upsert({
          where: { id: `${module.id}-lesson-${lessonData.order}` },
          update: {
            title: lessonData.title,
            content: lessonData.content,
            order: lessonData.order,
            aiGenerated: true, // Mark for AI video generation
            videoPrompt: `Enterprise-level educational video: ${lessonData.title}. ${moduleData.description}. Professional presentation with NeoVerse 4D world modeling, interactive instructor avatar, and VEO3 quality.`
          },
          create: {
            id: `${module.id}-lesson-${lessonData.order}`,
            title: lessonData.title,
            content: lessonData.content,
            order: lessonData.order,
            moduleId: module.id,
            aiGenerated: true,
            videoPrompt: `Enterprise-level educational video: ${lessonData.title}. ${moduleData.description}. Professional presentation with NeoVerse 4D world modeling, interactive instructor avatar, and VEO3 quality.`,
            language: 'python'
          }
        })

        console.log(`      📝 Lesson: ${lesson.title}`)
      }
    }

    console.log(`\n   ✅ Course "${course.title}" completed with ${courseData.modules.length} modules\n`)
  }

  console.log('='.repeat(60))
  console.log('✅ All Enterprise Courses Created Successfully!')
  console.log('='.repeat(60))
  console.log(`\n📊 Summary:`)
  console.log(`   - Learning Paths: 1`)
  console.log(`   - Courses: ${enterpriseCourses.length}`)
  console.log(`   - Total Modules: ${enterpriseCourses.reduce((acc, c) => acc + c.modules.length, 0)}`)
  console.log(`   - Total Lessons: ${enterpriseCourses.reduce((acc, c) => acc + c.modules.reduce((mAcc, m) => mAcc + m.lessons.length, 0), 0)}`)
  console.log(`\n🎬 Next Steps:`)
  console.log(`   1. Generate videos: bun run generate:enhanced-videos`)
  console.log(`   2. Review courses in admin panel`)
  console.log(`   3. Customize content as needed`)
  console.log(`   4. Deploy to production\n`)
}

createEnterpriseCourses()
  .catch(console.error)
  .finally(() => prisma.$disconnect())
