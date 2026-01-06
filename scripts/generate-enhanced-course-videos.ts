/**
 * Enhanced Course Video Generation Script
 * Uses NeoVerse, AvatarForcing, and VEO3 to create realistic robot training videos
 * 
 * Uses Gemini API models:
 * - Veo 3.1: Video generation with sound effects
 * - Gemini 3 Flash: Text generation and enhancement
 * - Gemini TTS: Text to speech for narration
 */

// Load environment variables
import { config } from 'dotenv'
config()

import { PrismaClient } from '@prisma/client'
import { enhancedVideoService } from '../src/lib/video-generation/enhanced-video-service'
import { getRobotTemplate, createVideoConfigFromTemplate, getPromptFromTemplate } from '../src/lib/video-generation/robot-templates'

const prisma = new PrismaClient()

// Robot type mapping for courses
const courseRobotMapping: Record<string, 'unitree-g1' | 'kabuki2' | 'generic'> = {
  'unitree-g1-fundamentals': 'unitree-g1',
  'ros2-ai-integration': 'kabuki2',
  'python-robotics': 'generic',
  'electronics-humanoid-robots': 'generic'
}

// Template type mapping based on lesson content
function determineTemplateType(lessonTitle: string, lessonContent: string): 'introduction' | 'demonstration' | 'technical' | 'tutorial' {
  const titleLower = lessonTitle.toLowerCase()
  const contentLower = lessonContent.toLowerCase()

  if (titleLower.includes('introduction') || titleLower.includes('overview') || titleLower.includes('basics')) {
    return 'introduction'
  }
  if (titleLower.includes('demonstration') || titleLower.includes('example') || titleLower.includes('demo')) {
    return 'demonstration'
  }
  if (titleLower.includes('tutorial') || titleLower.includes('step-by-step') || titleLower.includes('how to')) {
    return 'tutorial'
  }
  return 'technical'
}

async function generateVideoForLesson(
  lesson: any,
  robotType: 'unitree-g1' | 'kabuki2' | 'generic',
  courseSlug: string
) {
  try {
    console.log(`\n🎬 Generating enhanced video for: ${lesson.title}`)
    console.log(`🤖 Robot: ${robotType}`)
    console.log(`📚 Course: ${courseSlug}`)

    // Determine template type
    const templateType = determineTemplateType(lesson.title, lesson.content)
    console.log(`📋 Template type: ${templateType}`)

    // Get robot template
    const template = getRobotTemplate(robotType)
    
    // Get base prompt from template
    const basePrompt = getPromptFromTemplate(robotType, templateType, lesson.content.substring(0, 500))

    // Create video configuration
    const config = createVideoConfigFromTemplate(
      robotType,
      lesson.id,
      templateType,
      {
        // Override with lesson-specific content
        instructorAvatar: template.defaultConfig.instructorAvatar ? {
          ...template.defaultConfig.instructorAvatar,
          name: template.defaultConfig.instructorAvatar.name || 'Instructor'
        } : undefined
      }
    )

    // Generate video using enhanced service
    console.log(`🚀 Starting video generation with:`)
    console.log(`   - NeoVerse: ${config.useNeoVerse ? '✅' : '❌'}`)
    console.log(`   - AvatarForcing: ${config.useAvatarForcing ? '✅' : '❌'}`)
    console.log(`   - VEO3: ${config.useVEO3 ? '✅' : '❌'}`)

    // Update prompt with lesson-specific content
    const enhancedPrompt = `${basePrompt}\n\nLesson Content: ${lesson.content.substring(0, 1000)}`

    // Generate video with custom prompt
    const result = await enhancedVideoService.generateVideo(config, enhancedPrompt)

    if (result.success && result.taskId && result.videoId) {
      console.log(`✅ Video generation started!`)
      console.log(`📝 Task ID: ${result.taskId}`)
      console.log(`🆔 Video ID: ${result.videoId}`)

      // Poll for completion
      console.log(`⏳ Polling for video completion...`)
      let pollCount = 0
      const maxPolls = 60 // 10 minutes max
      const pollInterval = 10000 // 10 seconds

      while (pollCount < maxPolls) {
        await new Promise(resolve => setTimeout(resolve, pollInterval))
        pollCount++

        const statusResult = await enhancedVideoService.pollVideoStatus(result.taskId, result.videoId)

        if (statusResult.success && statusResult.videoUrl) {
          console.log(`🎉 Video generated successfully!`)
          console.log(`🔗 URL: ${statusResult.videoUrl}`)
          return statusResult.videoUrl
        } else if (statusResult.error) {
          console.log(`❌ Video generation failed: ${statusResult.error}`)
          return null
        }

        console.log(`⏳ Poll ${pollCount}/${maxPolls}: Still processing...`)
      }

      console.log(`⏸️ Video generation still in progress. Task ID: ${result.taskId}`)
      return null
    } else {
      console.log(`❌ Failed to start video generation: ${result.error}`)
      return null
    }
  } catch (error: any) {
    console.error(`❌ Error generating video for ${lesson.title}:`, error?.message || error)
    return null
  }
}

async function main() {
  try {
    console.log('🚀 Starting Enhanced Course Video Generation')
    console.log('📦 Using: NeoVerse + AvatarForcing + VEO3\n')

    // Get all courses
    const courses = await prisma.course.findMany({
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

    console.log(`📚 Found ${courses.length} courses\n`)

    const results: Array<{
      courseSlug: string
      lessonId: string
      lessonTitle: string
      success: boolean
      videoUrl?: string
    }> = []

    // Process each course
    for (const course of courses) {
      const robotType = courseRobotMapping[course.slug] || 'generic'
      console.log(`\n${'='.repeat(60)}`)
      console.log(`📚 Course: ${course.title}`)
      console.log(`🤖 Robot Type: ${robotType}`)
      console.log(`📦 Modules: ${course.modules.length}`)
      console.log(`${'='.repeat(60)}\n`)

      const allLessons = course.modules.flatMap(module => 
        module.lessons.map(lesson => ({ ...lesson, moduleTitle: module.title }))
      )

      // Generate videos for each lesson
      for (const lesson of allLessons) {
        // Skip if video already exists
        if (lesson.videoUrl) {
          console.log(`⏭️  Skipping ${lesson.title} - video already exists`)
          results.push({
            courseSlug: course.slug,
            lessonId: lesson.id,
            lessonTitle: lesson.title,
            success: true,
            videoUrl: lesson.videoUrl
          })
          continue
        }

        const videoUrl = await generateVideoForLesson(lesson, robotType, course.slug)

        results.push({
          courseSlug: course.slug,
          lessonId: lesson.id,
          lessonTitle: lesson.title,
          success: !!videoUrl,
          videoUrl: videoUrl || undefined
        })

        // Wait between requests to avoid rate limiting
        if (lesson !== allLessons[allLessons.length - 1]) {
          console.log(`\n⏸️  Waiting 5 seconds before next generation...\n`)
          await new Promise(resolve => setTimeout(resolve, 5000))
        }
      }
    }

    // Summary
    console.log('\n' + '='.repeat(60))
    console.log('📊 GENERATION SUMMARY')
    console.log('='.repeat(60))

    const successful = results.filter(r => r.success).length
    const failed = results.filter(r => !r.success).length

    console.log(`✅ Successful: ${successful}/${results.length}`)
    console.log(`❌ Failed: ${failed}/${results.length}`)

    // Group by course
    const byCourse: Record<string, typeof results> = {}
    results.forEach(r => {
      if (!byCourse[r.courseSlug]) {
        byCourse[r.courseSlug] = []
      }
      byCourse[r.courseSlug].push(r)
    })

    console.log('\n📚 Results by Course:')
    Object.entries(byCourse).forEach(([slug, courseResults]) => {
      const courseSuccess = courseResults.filter(r => r.success).length
      console.log(`\n  ${slug}:`)
      console.log(`    ✅ ${courseSuccess}/${courseResults.length} videos generated`)
    })

    if (successful > 0) {
      console.log('\n✅ Successfully generated videos:')
      results.filter(r => r.success).slice(0, 10).forEach(r => {
        console.log(`   - ${r.lessonTitle} (${r.courseSlug})`)
      })
      if (successful > 10) {
        console.log(`   ... and ${successful - 10} more`)
      }
    }

    if (failed > 0) {
      console.log('\n❌ Failed to generate videos:')
      results.filter(r => !r.success).forEach(r => {
        console.log(`   - ${r.lessonTitle} (${r.courseSlug})`)
      })
    }

    console.log('\n✨ Enhanced video generation process completed!')
    console.log('🎉 Videos created with NeoVerse 4D modeling, AvatarForcing avatars, and VEO3 quality!')
  } catch (error: any) {
    console.error('❌ Fatal error:', error?.message || error)
    process.exit(1)
  } finally {
    await prisma.$disconnect()
  }
}

main()
