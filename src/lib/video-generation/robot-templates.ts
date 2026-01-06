/**
 * Robot-Specific Video Generation Templates
 * Templates for Unitree G1, Kabuki2, and other robots with realistic settings
 */

import { VideoGenerationConfig } from './enhanced-video-service'

export interface RobotTemplate {
  robotType: string
  name: string
  description: string
  defaultConfig: Partial<VideoGenerationConfig>
  promptTemplates: {
    introduction: string
    demonstration: string
    technical: string
    tutorial: string
  }
}

export const robotTemplates: Record<string, RobotTemplate> = {
  'unitree-g1': {
    robotType: 'unitree-g1',
    name: 'Unitree G1',
    description: 'Advanced humanoid robot with dynamic balance and acrobatic capabilities',
    defaultConfig: {
      robotType: 'unitree-g1',
      useNeoVerse: true,
      useAvatarForcing: true,
      useVEO3: true,
      quality: 'high',
      resolution: '1080p',
      duration: 30,
      simulationEnvironment: {
        type: 'hybrid',
        lighting: 'studio',
        background: 'lab'
      },
      instructorAvatar: {
        name: 'Dr. Sarah Chen',
        style: 'technical',
        voiceProfile: 'voice-bella'
      }
    },
    promptTemplates: {
      introduction: `Professional introduction to the Unitree G1 humanoid robot. Show the robot from multiple angles in a modern robotics laboratory. Highlight key specifications: 1.8m height, 45kg weight, 32 degrees of freedom. Display technical diagrams overlaying the robot showing joint structure, sensor placement, and control systems. Professional presentation style with clear annotations.`,
      demonstration: `Unitree G1 robot performing dynamic movements: walking forward and backward, turning, balancing on one leg, performing acrobatic flips. Show the robot in a spacious lab environment with proper lighting. Include real-time telemetry overlays showing joint angles, center of mass position, and balance metrics. Smooth camera movements tracking the robot's motion.`,
      technical: `Technical deep-dive into Unitree G1's control systems. Show the robot's internal structure through cutaway animations, demonstrate balance control algorithms with visual feedback, display sensor data (IMU, force sensors, cameras) in real-time overlays. Professional technical presentation with detailed explanations.`,
      tutorial: `Step-by-step tutorial for programming the Unitree G1 robot. Show code editor with Python/ROS2 code, terminal with command outputs, and the robot executing the commands in real-time. Split-screen view showing code on one side and robot demonstration on the other. Clear annotations explaining each step.`
    }
  },
  'kabuki2': {
    robotType: 'kabuki2',
    name: 'Kabuki2',
    description: 'Gazebo-based humanoid robot with advanced manipulation capabilities',
    defaultConfig: {
      robotType: 'kabuki2',
      useNeoVerse: true,
      useAvatarForcing: true,
      useVEO3: true,
      quality: 'high',
      resolution: '1080p',
      duration: 30,
      simulationEnvironment: {
        type: 'gazebo',
        lighting: 'natural',
        background: 'lab'
      },
      instructorAvatar: {
        name: 'Prof. James Rodriguez',
        style: 'professional',
        voiceProfile: 'voice-adam'
      }
    },
    promptTemplates: {
      introduction: `Introduction to the Kabuki2 humanoid robot from Gazebo simulation. Show the robot in a realistic Gazebo simulation environment with proper physics, lighting, and materials. Display the robot's articulated joints, manipulation capabilities, and sensor systems. Professional presentation with technical specifications.`,
      demonstration: `Kabuki2 robot demonstrating manipulation tasks: picking up objects, placing items, performing gestures, and interacting with the environment. Show realistic physics simulation with proper object interactions, lighting, and shadows. Include Gazebo simulation interface overlays showing joint states and sensor readings.`,
      technical: `Technical analysis of Kabuki2's manipulation systems. Show detailed views of the robot's hands and arms, demonstrate inverse kinematics calculations, display force/torque sensor data, and show path planning for manipulation tasks. Gazebo simulation with realistic physics and materials.`,
      tutorial: `Tutorial for setting up and controlling Kabuki2 in Gazebo. Show Gazebo simulation interface, ROS2 command line tools, and the robot responding to commands. Split-screen showing code/commands and robot execution. Clear step-by-step instructions with annotations.`
    }
  },
  'generic': {
    robotType: 'generic',
    name: 'Generic Humanoid Robot',
    description: 'Standard humanoid robot template for general robotics education',
    defaultConfig: {
      robotType: 'generic',
      useNeoVerse: true,
      useAvatarForcing: false,
      useVEO3: true,
      quality: 'high',
      resolution: '1080p',
      duration: 30,
      simulationEnvironment: {
        type: 'real-world',
        lighting: 'studio',
        background: 'lab'
      }
    },
    promptTemplates: {
      introduction: `Introduction to humanoid robotics. Show a humanoid robot in a professional robotics laboratory setting. Highlight key components: joints, sensors, actuators, and control systems. Professional educational presentation style.`,
      demonstration: `Humanoid robot demonstrating basic movements: walking, turning, arm movements, and balance control. Show the robot in a clean lab environment with proper lighting. Include technical annotations explaining the movements.`,
      technical: `Technical explanation of humanoid robot systems. Show control algorithms, sensor integration, and motion planning. Professional technical presentation with diagrams and visualizations.`,
      tutorial: `Tutorial for programming humanoid robots. Show code examples, robot demonstrations, and step-by-step instructions. Professional educational presentation style.`
    }
  }
}

/**
 * Get template for a specific robot
 */
export function getRobotTemplate(robotType: string): RobotTemplate {
  return robotTemplates[robotType] || robotTemplates['generic']
}

/**
 * Generate video config from template
 */
export function createVideoConfigFromTemplate(
  robotType: string,
  lessonId: string,
  templateType: 'introduction' | 'demonstration' | 'technical' | 'tutorial',
  overrides?: Partial<VideoGenerationConfig>
): VideoGenerationConfig {
  const template = getRobotTemplate(robotType)
  const baseConfig = template.defaultConfig as VideoGenerationConfig

  return {
    ...baseConfig,
    lessonId,
    ...overrides
  }
}

/**
 * Get prompt for a specific robot and template type
 */
export function getPromptFromTemplate(
  robotType: string,
  templateType: 'introduction' | 'demonstration' | 'technical' | 'tutorial',
  customContent?: string
): string {
  const template = getRobotTemplate(robotType)
  const basePrompt = template.promptTemplates[templateType]
  
  if (customContent) {
    return `${basePrompt}\n\nAdditional content: ${customContent}`
  }
  
  return basePrompt
}
