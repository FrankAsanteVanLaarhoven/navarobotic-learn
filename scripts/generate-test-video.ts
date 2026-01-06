/**
 * Generate a single test video to check quality
 * This will create one video for a specific lesson to preview quality
 */

import { config } from 'dotenv'
config()

import { PrismaClient } from '@prisma/client'
import { enhancedVideoService } from '../src/lib/video-generation/enhanced-video-service'
import type { VideoGenerationConfig } from '../src/lib/video-generation/enhanced-video-service'

const prisma = new PrismaClient()

async function main() {
  try {
    console.log('🎬 Generating Test Video for Quality Check\n')
    console.log('='.repeat(80))

    // Find a good test lesson (preferably from Python 3 for Robotics)
    const testCourse = await prisma.course.findFirst({
      where: {
        slug: 'python-3-for-robotics'
      },
      include: {
        modules: {
          include: {
            lessons: {
              where: {
                videoUrl: null // No video yet
              },
              orderBy: { order: 'asc' },
              take: 1
            }
          },
          orderBy: { order: 'asc' }
        }
      }
    })

    if (!testCourse || !testCourse.modules[0]?.lessons[0]) {
      console.log('❌ No suitable test lesson found. Trying any course...')
      
      const anyLesson = await prisma.lesson.findFirst({
        where: {
          videoUrl: null
        },
        include: {
          module: {
            include: {
              course: true
            }
          }
        }
      })

      if (!anyLesson) {
        console.log('❌ No lessons without videos found!')
        process.exit(1)
      }

      await generateTestVideo(anyLesson)
      return
    }

    const testLesson = testCourse.modules[0].lessons[0]
    console.log(`📚 Course: ${testCourse.title}`)
    console.log(`📝 Lesson: ${testLesson.title}`)
    console.log(`📄 Content: ${testLesson.content.substring(0, 100)}...\n`)

    await generateTestVideo(testLesson)

  } catch (error: any) {
    console.error('❌ Error:', error?.message || error)
    process.exit(1)
  } finally {
    await prisma.$disconnect()
  }
}

async function generateTestVideo(lesson: any) {
  try {
    console.log('🚀 Starting video generation...\n')

    // Create enhanced video configuration
    const config: VideoGenerationConfig = {
      lessonId: lesson.id,
      robotType: 'generic',
      useNeoVerse: true, // 4D world modeling
      useAvatarForcing: true, // Interactive instructor avatar
      useVEO3: true, // Cinematic quality
      quality: 'high',
      duration: 30, // 30 seconds for test
      resolution: '1080p',
      instructorAvatar: {
        name: 'Dr. Sarah Chen',
        style: 'professional',
        voiceProfile: 'voice-bella'
      },
      simulationEnvironment: {
        type: 'hybrid',
        lighting: 'studio',
        background: 'lab'
      }
    }

    // Create comprehensive prompt
    const prompt = `World-class educational video: ${lesson.title}

${lesson.content.substring(0, 500)}

VIDEO STYLE: Professional instructor-led course with step-by-step walkthrough, interactive coding workshop, and comprehensive explanations covering WHAT, WHY, HOW, WHEN, WHERE, and WHO. Real AI voice narration, split-screen showing instructor and code workspace, live coding demonstrations, and robot simulation integration.

QUALITY REQUIREMENTS:
- Professional cinematography
- Clear technical demonstrations
- High-fidelity visual quality
- Smooth camera movements
- Realistic robot movements
- Professional educational presentation style`

    console.log('📋 Configuration:')
    console.log(`   🤖 Robot Type: ${config.robotType}`)
    console.log(`   🎬 NeoVerse 4D: ${config.useNeoVerse ? '✅' : '❌'}`)
    console.log(`   👤 AvatarForcing: ${config.useAvatarForcing ? '✅' : '❌'}`)
    console.log(`   🎥 Veo 3.1: ${config.useVEO3 ? '✅' : '❌'}`)
    console.log(`   📐 Resolution: ${config.resolution}`)
    console.log(`   ⏱️  Duration: ${config.duration}s`)
    console.log(`   🎨 Quality: ${config.quality}\n`)

    // Generate video
    console.log('🎬 Generating video with Gemini API...\n')
    const result = await enhancedVideoService.generateVideo(config, prompt)

    if (!result.success) {
      console.error('❌ Video generation failed:', result.error)
      process.exit(1)
    }

    if (!result.taskId || !result.videoId) {
      console.error('❌ No task ID or video ID returned')
      process.exit(1)
    }

    console.log('✅ Video generation started!')
    console.log(`📝 Task ID: ${result.taskId}`)
    console.log(`🆔 Video ID: ${result.videoId}`)
    console.log(`🎨 Features: ${result.metadata?.features.join(', ')}\n`)

    console.log('⏳ Polling for video completion...')
    console.log('   (This may take 5-10 minutes)\n')

    // Poll for completion
    let pollCount = 0
    const maxPolls = 120 // 20 minutes max
    const pollInterval = 10000 // 10 seconds

    while (pollCount < maxPolls) {
      await new Promise(resolve => setTimeout(resolve, pollInterval))
      pollCount++

      const statusResult = await enhancedVideoService.pollVideoStatus(result.taskId, result.videoId)

      if (statusResult.success && statusResult.videoUrl) {
        console.log('\n' + '='.repeat(80))
        console.log('🎉 VIDEO GENERATION COMPLETE!')
        console.log('='.repeat(80))
        console.log(`\n✅ Video URL: ${statusResult.videoUrl}`)
        console.log(`\n📚 Course: ${lesson.module.course.title}`)
        console.log(`📝 Lesson: ${lesson.title}`)
        console.log(`\n🌐 Access via:`)
        console.log(`   Web UI: http://localhost:3000/courses/${lesson.module.course.slug}`)
        console.log(`   Direct URL: ${statusResult.videoUrl}`)
        console.log(`\n💡 To view:`)
        console.log(`   1. Start dev server: bun run dev`)
        console.log(`   2. Go to: http://localhost:3000/courses/${lesson.module.course.slug}`)
        console.log(`   3. Click on the lesson: ${lesson.title}`)
        console.log(`   4. Click the "Video" tab`)
        console.log('\n✨ Check the quality and approve before generating the rest!')
        return
      }

      if (statusResult.error && !statusResult.error.includes('still processing')) {
        console.error(`\n❌ Error: ${statusResult.error}`)
        break
      }

      // Show progress
      const video = await prisma.generatedVideo.findUnique({
        where: { id: result.videoId }
      })

      if (video) {
        process.stdout.write(`\r⏳ Progress: ${video.progress.toFixed(1)}% | Status: ${video.status} | Poll: ${pollCount}/${maxPolls}`)
      }
    }

    console.log('\n\n⏳ Video is still processing. Check status with:')
    console.log(`   bun run check:videos`)
    console.log(`\n📝 Task ID: ${result.taskId}`)
    console.log(`🆔 Video ID: ${result.videoId}`)

  } catch (error: any) {
    console.error('\n❌ Error generating test video:', error?.message || error)
    throw error
  }
}

main()
