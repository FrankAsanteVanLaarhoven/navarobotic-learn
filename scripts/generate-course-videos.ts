import { PrismaClient } from '@prisma/client'
import { geminiVideoService } from '../src/lib/video-generation/gemini-video-service'
import type { VideoGenerationConfig } from '../src/lib/video-generation/enhanced-video-service'

const prisma = new PrismaClient()

// Lesson-specific prompts for video generation
// Optimized for Gemini 2.5 with Veo 3 video generation
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

async function generateVideoForLesson(lessonId: string, prompt: string, lessonTitle: string) {
  try {
    console.log(`\n🎬 Generating video for: ${lessonTitle}`)
    console.log(`📝 Prompt: ${prompt.substring(0, 100)}...`)

    // Enhanced prompt optimized for Veo 3.1 generation
    const enhancedPrompt = `${prompt} High quality educational video, professional cinematography, clear technical demonstrations, realistic robot movements, suitable for online learning platform. Cinematic lighting, smooth camera movements, detailed technical annotations.`

    // Use Gemini video service with Veo 3.1
    const config: VideoGenerationConfig = {
      lessonId: lessonId,
      robotType: 'generic',
      useNeoVerse: true,
      useAvatarForcing: true,
      useVEO3: true,
      quality: 'high',
      duration: 30,
      resolution: '1080p'
    }

    const result = await geminiVideoService.generateVideo(enhancedPrompt, config)

    if (!result.success || !result.taskId || !result.videoId) {
      console.log(`❌ Video generation failed: ${result.error}`)
      return null
    }

    console.log(`✅ Task created! Task ID: ${result.taskId}`)
    console.log(`🤖 Model: Veo 3.1 (via Gemini API)`)

    // Poll for result
    let pollCount = 0
    const maxPolls = 60 // Poll for up to 10 minutes (60 * 10 seconds)
    const pollInterval = 10000 // 10 seconds

    while (pollCount < maxPolls) {
      pollCount++
      const statusResult = await geminiVideoService.pollVideoStatus(result.taskId, result.videoId)
      
      if (statusResult.success && statusResult.videoUrl) {
        console.log(`🎉 Video generated successfully!`)
        console.log(`🔗 URL: ${statusResult.videoUrl}`)

        // Update lesson in database
        await prisma.lesson.update({
          where: { id: lessonId },
          data: { videoUrl: statusResult.videoUrl }
        })

        console.log(`💾 Database updated with video URL`)
        return statusResult.videoUrl
      }

      if (statusResult.error && !statusResult.error.includes('still processing')) {
        console.log(`❌ Video generation failed: ${statusResult.error}`)
        return null
      }

      const progress = Math.min(100, (pollCount / maxPolls) * 100)
      console.log(`⏳ Poll ${pollCount}/${maxPolls}: Still processing... (${Math.round(progress)}% estimated)`)
      await new Promise(resolve => setTimeout(resolve, pollInterval))
    }

    console.log(`⏸️ Task still processing. Task ID: ${result.taskId}`)
    console.log(`💡 You can query this task later`)
    return null
  } catch (error: any) {
    console.error(`❌ Error generating video for ${lessonTitle}:`, error?.message || error)
    return null
  }
}

async function main() {
  try {
    console.log('🚀 Starting video generation for Unitree G1 Fundamentals course...\n')

    // Fetch all lessons for the Unitree G1 course
    const course = await prisma.course.findUnique({
      where: { slug: 'unitree-g1-fundamentals' },
      include: {
        modules: {
          include: {
            lessons: {
              orderBy: { order: 'asc' }
            }
          },
          orderBy: { order: 'asc' }
        }
      }
    })

    if (!course) {
      console.error('❌ Course not found!')
      process.exit(1)
    }

    console.log(`📚 Found course: ${course.title}`)
    console.log(`📦 Modules: ${course.modules.length}`)
    console.log(`📝 Total lessons: ${course.modules.reduce((acc, m) => acc + m.lessons.length, 0)}`)

    const allLessons = course.modules.flatMap(module => 
      module.lessons.map(lesson => ({ ...lesson, moduleTitle: module.title }))
    )

    console.log(`\n🎥 Generating videos for ${allLessons.length} lessons...\n`)

    const results: Array<{ lessonId: string; title: string; success: boolean; videoUrl?: string }> = []

    // Generate videos for each lesson
    for (const lesson of allLessons) {
      const prompt = lessonPrompts[lesson.id] || `Educational video about ${lesson.title}. Professional presentation style, clear explanations, technical content about robotics and the Unitree G1 humanoid robot.`
      
      const videoUrl = await generateVideoForLesson(lesson.id, prompt, lesson.title)
      
      results.push({
        lessonId: lesson.id,
        title: lesson.title,
        success: !!videoUrl,
        videoUrl: videoUrl || undefined
      })

      // Wait a bit between requests to avoid rate limiting
      if (lesson !== allLessons[allLessons.length - 1]) {
        console.log(`\n⏸️ Waiting 5 seconds before next generation...\n`)
        await new Promise(resolve => setTimeout(resolve, 5000))
      }
    }

    // Summary
    console.log('\n' + '='.repeat(60))
    console.log('📊 GENERATION SUMMARY')
    console.log('='.repeat(60))
    
    const successful = results.filter(r => r.success).length
    const failed = results.filter(r => !r.success).length

    console.log(`✅ Successful: ${successful}/${allLessons.length}`)
    console.log(`❌ Failed: ${failed}/${allLessons.length}`)

    if (successful > 0) {
      console.log('\n✅ Successfully generated videos:')
      results.filter(r => r.success).forEach(r => {
        console.log(`   - ${r.title}`)
        console.log(`     URL: ${r.videoUrl}`)
      })
    }

    if (failed > 0) {
      console.log('\n❌ Failed to generate videos:')
      results.filter(r => !r.success).forEach(r => {
        console.log(`   - ${r.title}`)
      })
    }

    console.log('\n✨ Video generation process completed!')
  } catch (error: any) {
    console.error('❌ Fatal error:', error?.message || error)
    process.exit(1)
  } finally {
    await prisma.$disconnect()
  }
}

main()
