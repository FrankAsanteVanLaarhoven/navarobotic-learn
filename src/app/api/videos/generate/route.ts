import { NextResponse } from 'next/server'
import { db } from '@/lib/db'
import ZAI from 'z-ai-web-dev-sdk'

// Lesson-specific prompts for video generation
const lessonPrompts: Record<string, string> = {
  'lesson-g1-1-1': 'Professional educational video showing the Unitree G1 humanoid robot from multiple angles, highlighting its specifications, hardware components, degrees of freedom, and capabilities. Clean studio lighting, technical diagrams overlaying the robot, professional presentation style.',
  'lesson-g1-1-2': 'Step-by-step tutorial video showing how to set up the development environment for Unitree G1 robot programming. Screen recordings of code editor, terminal commands, SDK installation, ROS2 workspace configuration. Professional educational style with clear annotations.',
  'lesson-g1-1-3': 'Safety demonstration video for Unitree G1 robot control. Showing proper power-on procedures, basic movement commands, emergency stop protocols. Professional robotics lab setting, clear safety indicators, step-by-step instructions.',
  'lesson-g1-2-1': 'Educational animation explaining bipedal locomotion fundamentals. 3D animated humanoid robot walking, showing center of mass movement, support polygons, gait phases. Technical diagrams explaining physics of humanoid walking, professional educational style.',
  'lesson-g1-2-2': 'Technical video demonstrating balance control algorithms for humanoid robots. Real-time IMU data visualization, PID controller implementation, balance correction demonstrations. Unitree G1 robot maintaining balance on uneven surfaces, professional robotics lab setting.',
  'lesson-g1-2-3': 'Video showing different walking patterns for humanoid robots: forward walking, backward walking, turning, side-stepping. Unitree G1 robot demonstrating each gait pattern smoothly, technical annotations showing joint angles and trajectories, professional presentation.',
  'lesson-g1-3-1': 'Educational video explaining path planning algorithms for robotics. Animated visualizations of A* pathfinding, RRT algorithms, showing path computation on grid maps. 3D visualization of robot navigating through obstacles, professional technical presentation style.',
  'lesson-g1-3-2': 'Real-time demonstration of obstacle avoidance for Unitree G1 robot. LiDAR visualization showing detected obstacles, dynamic path replanning, reactive navigation behaviors. Professional robotics lab setting, clear technical annotations.',
  'lesson-g1-3-3': 'Technical video explaining SLAM (Simultaneous Localization and Mapping) for humanoid robots. Real-time map building visualization, landmark detection, odometry integration. Unitree G1 robot creating a map while navigating, professional educational presentation.',
  'lesson-g1-4-1': 'Video demonstrating vision systems integration for Unitree G1 robot. Camera calibration procedures, image processing pipelines, object detection demonstrations. Real-time computer vision overlays, professional technical tutorial style.',
  'lesson-g1-4-2': 'Advanced technical video showing sensor fusion techniques for robotics. Combining camera, LiDAR, and IMU data, Kalman filtering visualizations, unified perception system demonstrations. Professional robotics lab, clear technical explanations.',
  'lesson-g1-4-3': 'Educational video on environmental understanding for humanoid robots. Semantic segmentation demonstrations, scene analysis, context-aware navigation. Unitree G1 robot understanding and responding to different environments, professional presentation style.',
}

export async function POST(request: Request) {
  try {
    const { lessonId, model = 'high' } = await request.json()

    if (!lessonId) {
      return NextResponse.json(
        { error: 'Lesson ID is required' },
        { status: 400 }
      )
    }

    // Fetch lesson
    const lesson = await db.lesson.findUnique({
      where: { id: lessonId },
      include: {
        module: {
          include: {
            course: true
          }
        }
      }
    })

    if (!lesson) {
      return NextResponse.json(
        { error: 'Lesson not found' },
        { status: 404 }
      )
    }

    // Get prompt for this lesson
    const prompt = lessonPrompts[lessonId] || 
      `Educational video about ${lesson.title}. Professional presentation style, clear explanations, technical content about robotics and the Unitree G1 humanoid robot.`

    console.log(`🎬 Generating video for lesson: ${lesson.title}`)
    console.log(`🤖 Using Gemini 2.5 with Veo 3 model`)

    // Initialize ZAI (which supports Veo 3 via Gemini)
    const zai = await ZAI.create()

    // Enhanced prompt optimized for Gemini/Veo 3
    const enhancedPrompt = `${prompt} High quality educational video, professional cinematography, clear technical demonstrations, suitable for online learning platform.`

    // Create video generation task with Veo 3 settings
    const task = await zai.video.generations.create({
      prompt: enhancedPrompt,
      quality: model === 'high' ? 'high' : 'speed',
      with_audio: false, // Can be enabled for synchronized audio
      size: '1920x1080', // Full HD
      fps: 30,
      duration: 30, // 30 seconds (Veo 3 supports up to 8 seconds per clip, but can chain)
      model: 'veo' // Use Veo 3 model
    })

    // Create GeneratedVideo record
    const videoModel = await db.videoGenerationModel.findFirst({
      where: { name: 'Sora' } // Default to Sora
    })

    const generatedVideo = await db.generatedVideo.create({
      data: {
        title: `Video for ${lesson.title}`,
        description: `AI-generated video for lesson: ${lesson.title}`,
        videoUrl: '', // Will be updated when ready
        prompt: prompt,
        modelId: videoModel?.id || '',
        lessonId: lessonId,
        status: 'processing',
        progress: 0,
      }
    })

    // Return task info (client will poll for status)
    return NextResponse.json({
      success: true,
      taskId: task.id,
      videoId: generatedVideo.id,
      status: task.task_status,
      message: 'Video generation started. Poll /api/videos/status/[taskId] for updates.'
    })

  } catch (error: any) {
    console.error('Error generating video:', error)
    return NextResponse.json(
      { 
        error: 'Failed to generate video',
        message: error?.message || 'Unknown error'
      },
      { status: 500 }
    )
  }
}
