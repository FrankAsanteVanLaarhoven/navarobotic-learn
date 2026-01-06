# 🎬 Enhanced Video Generation with NeoVerse, AvatarForcing, and VEO3

This document explains how to use the enhanced video generation system that integrates **NeoVerse** (4D world modeling), **AvatarForcing** (interactive instructor avatars), and **VEO3** (high-quality video generation) to create realistic robot training videos.

## 🚀 Overview

The enhanced video generation system combines three cutting-edge AI models:

1. **NeoVerse** - 4D world model for realistic spatial-temporal modeling
2. **AvatarForcing** - Real-time interactive head avatar generation for instructors
3. **VEO3** - High-quality video generation with cinematic quality

## 📋 Features

### NeoVerse 4D World Modeling
- **Temporal Consistency**: Smooth motion across frames with 4D spatial-temporal modeling
- **Spatial Modeling**: Realistic 3D environment with depth and spatial awareness
- **Real-World Physics**: Accurate gravity, friction, and material interactions
- **Camera Tracking**: Cinematic camera with 3D spatial awareness, smooth tracking
- **Global Illumination**: Realistic lighting with shadows and reflections

### AvatarForcing Interactive Avatars
- **Real-Time Generation**: ~500ms latency for interactive feel
- **Natural Reactions**: Expressive facial expressions and head movements
- **Synchronized Lip-Sync**: Perfect audio-visual synchronization
- **Content Responsive**: Avatar reacts to robot demonstrations with appropriate expressions
- **Professional Presentation**: Suitable for educational content

### VEO3 Video Generation
- **Cinematic Quality**: 4K resolution support
- **Realistic Movements**: Temporal consistency in robot motions
- **Professional Cinematography**: Smooth camera movements
- **High Fidelity**: Suitable for professional educational content

## 🎯 Usage

### Command Line Script

Generate enhanced videos for all courses:

```bash
bun run generate:enhanced-videos
```

This script will:
- ✅ Process all courses in the database
- ✅ Use robot-specific templates (Unitree G1, Kabuki2, etc.)
- ✅ Generate videos with NeoVerse, AvatarForcing, and VEO3
- ✅ Update lesson records with video URLs
- ✅ Provide detailed progress and summary

### API Endpoint

Generate a single video via API:

```bash
POST /api/videos/generate-enhanced
Content-Type: application/json

{
  "lessonId": "lesson-g1-1-1",
  "robotType": "unitree-g1",
  "useNeoVerse": true,
  "useAvatarForcing": true,
  "useVEO3": true,
  "quality": "high",
  "duration": 30,
  "resolution": "1080p",
  "instructorAvatar": {
    "name": "Dr. Sarah Chen",
    "style": "technical",
    "voiceProfile": "voice-bella"
  },
  "simulationEnvironment": {
    "type": "hybrid",
    "lighting": "studio",
    "background": "lab"
  }
}
```

### Check Video Status

```bash
GET /api/videos/generate-enhanced?taskId=<taskId>&videoId=<videoId>
```

## 🤖 Robot Templates

### Unitree G1
- **Template**: Optimized for humanoid robot demonstrations
- **Instructor**: Dr. Sarah Chen (technical style)
- **Environment**: Hybrid (simulation + real-world)
- **Features**: Dynamic balance, acrobatics, walking control

### Kabuki2
- **Template**: Optimized for Gazebo simulation
- **Instructor**: Prof. James Rodriguez (professional style)
- **Environment**: Gazebo simulation
- **Features**: Manipulation, object handling, gestures

### Generic
- **Template**: General humanoid robotics
- **Environment**: Real-world lab setting
- **Features**: Basic movements, control systems

## 📝 Template Types

Each robot template supports four video types:

1. **Introduction** - Robot overview and specifications
2. **Demonstration** - Robot performing tasks and movements
3. **Technical** - Deep-dive into control systems and algorithms
4. **Tutorial** - Step-by-step programming guides

## 🎨 Configuration Options

### Video Generation Config

```typescript
{
  lessonId: string
  robotType: 'unitree-g1' | 'kabuki2' | 'generic'
  useNeoVerse?: boolean        // Enable 4D world modeling
  useAvatarForcing?: boolean    // Enable instructor avatar
  useVEO3?: boolean            // Use VEO3 for generation
  quality: 'high' | 'speed'    // Video quality
  duration: number              // Duration in seconds
  resolution: '720p' | '1080p' | '4K'
  instructorAvatar?: {
    name: string
    style: 'professional' | 'casual' | 'technical'
    voiceProfile?: string
  }
  simulationEnvironment?: {
    type: 'gazebo' | 'real-world' | 'hybrid'
    lighting: 'studio' | 'natural' | 'lab'
    background: 'lab' | 'outdoor' | 'indoor' | 'custom'
  }
}
```

## 🔧 Integration with Simulations

### Using NeoVerse in 3D Simulations

The `NeoVerseSimulation` component enhances your 3D robot simulations:

```tsx
import NeoVerseSimulation from '@/components/simulation/NeoVerseSimulation'

<NeoVerseSimulation
  selectedRobot="unitree-g1"
  isRunning={true}
  speed={1.0}
  jointAngles={[45, 45, 30, 30, 0, 0, 0, 0]}
  config={{
    enabled: true,
    temporalConsistency: true,
    spatialModeling: true,
    realWorldPhysics: true,
    cameraTracking: true
  }}
/>
```

## 📊 Video Generation Process

1. **Prompt Generation**
   - Robot-specific template selection
   - NeoVerse 4D world modeling description
   - AvatarForcing instructor avatar configuration
   - VEO3 quality settings

2. **Video Generation**
   - Task creation via Gemini API (Veo 3.1)
   - VEO3 model selection
   - Quality and resolution settings

3. **Status Polling**
   - Automatic polling every 10 seconds
   - Progress updates in database
   - Video URL retrieval on completion

4. **Database Update**
   - GeneratedVideo record creation
   - Lesson videoUrl update
   - Status tracking

## 🎓 Example: Generating Unitree G1 Course Videos

```typescript
import { enhancedVideoService } from '@/lib/video-generation/enhanced-video-service'
import { createVideoConfigFromTemplate } from '@/lib/video-generation/robot-templates'

// Create config for Unitree G1 introduction video
const config = createVideoConfigFromTemplate(
  'unitree-g1',
  'lesson-g1-1-1',
  'introduction'
)

// Generate video
const result = await enhancedVideoService.generateVideo(config)

if (result.success) {
  // Poll for completion
  const finalResult = await enhancedVideoService.pollVideoStatus(
    result.taskId!,
    result.videoId!
  )
  
  console.log('Video URL:', finalResult.videoUrl)
}
```

## 🔍 Monitoring and Debugging

### Check Generation Status

```bash
# Check specific video
GET /api/videos/status/<taskId>

# Check all videos for a lesson
GET /api/videos?lessonId=<lessonId>
```

### Database Queries

```typescript
// Get all generated videos
const videos = await db.generatedVideo.findMany({
  where: { lessonId: 'lesson-g1-1-1' },
  include: { model: true, lesson: true }
})

// Check generation status
const status = await db.generatedVideo.findUnique({
  where: { id: 'video-id' }
})
```

## 🎨 Customization

### Custom Robot Templates

Add new robot templates in `src/lib/video-generation/robot-templates.ts`:

```typescript
export const robotTemplates: Record<string, RobotTemplate> = {
  'your-robot': {
    robotType: 'your-robot',
    name: 'Your Robot Name',
    description: 'Robot description',
    defaultConfig: { /* ... */ },
    promptTemplates: { /* ... */ }
  }
}
```

### Custom Instructor Avatars

Configure instructor avatars:

```typescript
const config = {
  instructorAvatar: {
    name: 'Your Instructor Name',
    style: 'professional', // or 'casual', 'technical'
    voiceProfile: 'voice-adam' // or 'voice-bella'
  }
}
```

## 📈 Performance

- **Generation Time**: 1-10 minutes per video (depending on duration and quality)
- **Latency**: AvatarForcing achieves ~500ms for real-time feel
- **Quality**: VEO3 provides cinematic quality at 1080p/4K
- **Rate Limiting**: 5-second delays between requests (configurable)

## 🐛 Troubleshooting

### Video Generation Fails

1. Check Gemini API credentials
2. Verify internet connection
3. Check API rate limits
4. Review error messages in console
5. Try generating individual lessons

### Avatar Not Appearing

1. Ensure `useAvatarForcing: true`
2. Check instructor avatar configuration
3. Verify voice profile exists
4. Check video generation logs

### NeoVerse Not Working

1. Ensure `useNeoVerse: true` in config
2. Check simulation environment settings
3. Verify 3D scene setup
4. Check browser console for errors

## 📚 References

- [NeoVerse GitHub](https://github.com/IamCreateAI/NeoVerse) - 4D World Model
- [AvatarForcing GitHub](https://github.com/TaekyungKi/AvatarForcing) - Interactive Avatars
- [VEO3 Documentation](https://ai.google.dev/gemini-api/docs/video-generation) - Video Generation

## 🎉 Success Indicators

After successful generation:
- ✅ Video URLs in lesson records
- ✅ GeneratedVideo records with 'completed' status
- ✅ Videos playable in course pages
- ✅ No "Video coming soon" placeholders
- ✅ High-quality videos with realistic robot movements
- ✅ Interactive instructor avatars (if enabled)
- ✅ Realistic 4D world modeling (if enabled)

---

**Built with ❤️ for realistic robot training videos**
