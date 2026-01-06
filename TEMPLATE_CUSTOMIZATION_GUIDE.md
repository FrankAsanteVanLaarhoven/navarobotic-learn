# 🎨 Template Customization Guide

This guide shows you how to customize robot templates for your specific use cases.

## 📋 Overview

Templates are defined in `src/lib/video-generation/robot-templates.ts` and include:
- Robot specifications
- Default configurations
- Prompt templates for different video types

## 🔧 Customizing Existing Templates

### Example: Customize Unitree G1 Template

```typescript
// In src/lib/video-generation/robot-templates.ts

export const robotTemplates: Record<string, RobotTemplate> = {
  'unitree-g1': {
    robotType: 'unitree-g1',
    name: 'Unitree G1',
    description: 'Your custom description here',
    defaultConfig: {
      robotType: 'unitree-g1',
      useNeoVerse: true,
      useAvatarForcing: true,
      useVEO3: true,
      quality: 'high',
      resolution: '1080p',
      duration: 30,
      simulationEnvironment: {
        type: 'hybrid', // or 'gazebo', 'real-world'
        lighting: 'studio', // or 'natural', 'lab'
        background: 'lab' // or 'outdoor', 'indoor', 'custom'
      },
      instructorAvatar: {
        name: 'Your Instructor Name',
        style: 'professional', // or 'casual', 'technical'
        voiceProfile: 'voice-adam' // or 'voice-bella'
      }
    },
    promptTemplates: {
      introduction: 'Your custom introduction prompt...',
      demonstration: 'Your custom demonstration prompt...',
      technical: 'Your custom technical prompt...',
      tutorial: 'Your custom tutorial prompt...'
    }
  }
}
```

## 🆕 Creating New Robot Templates

### Step 1: Add Template Definition

```typescript
'your-robot-name': {
  robotType: 'your-robot-name',
  name: 'Your Robot Name',
  description: 'Description of your robot',
  defaultConfig: {
    robotType: 'your-robot-name',
    useNeoVerse: true,
    useAvatarForcing: true,
    useVEO3: true,
    quality: 'high',
    resolution: '1080p',
    duration: 30,
    simulationEnvironment: {
      type: 'gazebo', // Choose appropriate type
      lighting: 'studio',
      background: 'lab'
    },
    instructorAvatar: {
      name: 'Instructor Name',
      style: 'professional',
      voiceProfile: 'voice-adam'
    }
  },
  promptTemplates: {
    introduction: `Professional introduction to Your Robot Name. 
      Show the robot from multiple angles. 
      Highlight key specifications: [your specs here].`,
    
    demonstration: `Your Robot Name performing [specific tasks]. 
      Show the robot in a [environment description]. 
      Include [specific features to highlight].`,
    
    technical: `Technical deep-dive into Your Robot Name's [systems]. 
      Show [technical details]. 
      Demonstrate [specific capabilities].`,
    
    tutorial: `Step-by-step tutorial for [programming/controlling] Your Robot Name. 
      Show [code/commands] and robot execution. 
      Clear step-by-step instructions.`
  }
}
```

### Step 2: Add to Course Mapping

```typescript
// In scripts/generate-enhanced-course-videos.ts

const courseRobotMapping: Record<string, 'unitree-g1' | 'kabuki2' | 'generic' | 'your-robot-name'> = {
  'unitree-g1-fundamentals': 'unitree-g1',
  'ros2-ai-integration': 'kabuki2',
  'your-course-slug': 'your-robot-name', // Add your course
  'python-robotics': 'generic',
  'electronics-humanoid-robots': 'generic'
}
```

## 🎯 Customizing Prompt Templates

### Introduction Template

Focus on:
- Robot overview and specifications
- Key features and capabilities
- Visual presentation style

Example:
```typescript
introduction: `Professional introduction to [Robot Name]. 
  Show the robot from multiple angles in a modern robotics laboratory. 
  Highlight key specifications: [height, weight, DOF, etc.]. 
  Display technical diagrams overlaying the robot showing [joint structure, sensors, etc.]. 
  Professional presentation style with clear annotations.`
```

### Demonstration Template

Focus on:
- Specific robot movements/tasks
- Real-time demonstrations
- Technical annotations

Example:
```typescript
demonstration: `[Robot Name] performing [specific tasks]: 
  [task 1], [task 2], [task 3]. 
  Show the robot in a [environment] with proper lighting. 
  Include real-time telemetry overlays showing [metrics]. 
  Smooth camera movements tracking the robot's motion.`
```

### Technical Template

Focus on:
- Deep technical analysis
- System architecture
- Algorithm demonstrations

Example:
```typescript
technical: `Technical deep-dive into [Robot Name]'s [systems]. 
  Show the robot's [internal structure] through [visualizations]. 
  Demonstrate [algorithms/control systems] with [visual feedback]. 
  Display [sensor data/telemetry] in real-time overlays. 
  Professional technical presentation with detailed explanations.`
```

### Tutorial Template

Focus on:
- Step-by-step instructions
- Code/command examples
- Clear annotations

Example:
```typescript
tutorial: `Step-by-step tutorial for [programming/controlling] [Robot Name]. 
  Show [code editor/terminal] with [language/framework] code. 
  Split-screen view showing code on one side and robot demonstration on the other. 
  Clear annotations explaining each step. 
  Professional educational presentation style.`
```

## 🎨 Customizing Instructor Avatars

### Avatar Styles

**Professional:**
```typescript
instructorAvatar: {
  name: 'Dr. [Name]',
  style: 'professional',
  voiceProfile: 'voice-adam'
}
```

**Casual:**
```typescript
instructorAvatar: {
  name: '[First Name]',
  style: 'casual',
  voiceProfile: 'voice-bella'
}
```

**Technical:**
```typescript
instructorAvatar: {
  name: 'Prof. [Name]',
  style: 'technical',
  voiceProfile: 'voice-adam'
}
```

## 🌍 Customizing Simulation Environments

### Environment Types

**Gazebo:**
```typescript
simulationEnvironment: {
  type: 'gazebo',
  lighting: 'natural',
  background: 'lab'
}
```

**Real-World:**
```typescript
simulationEnvironment: {
  type: 'real-world',
  lighting: 'studio',
  background: 'lab'
}
```

**Hybrid:**
```typescript
simulationEnvironment: {
  type: 'hybrid',
  lighting: 'studio',
  background: 'lab'
}
```

### Lighting Options
- `studio` - Professional studio lighting
- `natural` - Natural daylight
- `lab` - Laboratory lighting

### Background Options
- `lab` - Robotics laboratory
- `outdoor` - Outdoor environment
- `indoor` - Indoor setting
- `custom` - Custom background

## 📝 Example: Complete Custom Template

```typescript
'custom-robot': {
  robotType: 'custom-robot',
  name: 'Custom Humanoid Robot',
  description: 'A custom humanoid robot for educational purposes',
  defaultConfig: {
    robotType: 'custom-robot',
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
      name: 'Dr. Jane Smith',
      style: 'technical',
      voiceProfile: 'voice-bella'
    }
  },
  promptTemplates: {
    introduction: `Professional introduction to Custom Humanoid Robot. 
      Show the robot from multiple angles in a modern robotics laboratory. 
      Highlight key specifications: 1.5m height, 35kg weight, 24 degrees of freedom. 
      Display technical diagrams showing joint structure and sensor placement. 
      Professional presentation style with clear annotations.`,
    
    demonstration: `Custom Humanoid Robot performing basic movements: 
      walking forward, turning, arm gestures, and balance control. 
      Show the robot in a spacious lab environment with proper lighting. 
      Include real-time telemetry overlays showing joint angles and balance metrics. 
      Smooth camera movements tracking the robot's motion.`,
    
    technical: `Technical deep-dive into Custom Humanoid Robot's control systems. 
      Show the robot's internal structure through cutaway animations. 
      Demonstrate balance control algorithms with visual feedback. 
      Display sensor data (IMU, force sensors) in real-time overlays. 
      Professional technical presentation with detailed explanations.`,
    
    tutorial: `Step-by-step tutorial for programming Custom Humanoid Robot. 
      Show code editor with Python/ROS2 code, terminal with command outputs. 
      Split-screen view showing code on one side and robot demonstration on the other. 
      Clear annotations explaining each step. 
      Professional educational presentation style.`
  }
}
```

## 🔄 Using Custom Templates

### In Scripts

```typescript
import { createVideoConfigFromTemplate } from '@/lib/video-generation/robot-templates'

const config = createVideoConfigFromTemplate(
  'custom-robot', // Your robot type
  'lesson-id',
  'introduction' // Template type
)
```

### Via API

```json
{
  "lessonId": "lesson-id",
  "robotType": "custom-robot",
  "useNeoVerse": true,
  "useAvatarForcing": true,
  "useVEO3": true
}
```

## 💡 Best Practices

1. **Be Specific**: Include exact specifications and capabilities
2. **Use Technical Terms**: Include relevant technical terminology
3. **Visual Descriptions**: Describe camera angles and movements
4. **Environment Details**: Specify lighting, background, and setting
5. **Instructor Style**: Match instructor style to content type
6. **Test Prompts**: Generate test videos to refine prompts

## 🎬 Prompt Writing Tips

- **Start with action**: "Professional introduction to..." or "Step-by-step tutorial..."
- **Include specifications**: Height, weight, DOF, capabilities
- **Describe visuals**: Camera angles, lighting, environment
- **Add technical details**: Sensors, algorithms, control systems
- **Specify style**: Professional, educational, technical
- **Include annotations**: Diagrams, overlays, telemetry

## 📚 Next Steps

1. Customize templates for your robots
2. Test with sample lessons
3. Refine prompts based on results
4. Add to course mapping
5. Generate videos for all courses

---

**Happy Customizing! 🎉**
