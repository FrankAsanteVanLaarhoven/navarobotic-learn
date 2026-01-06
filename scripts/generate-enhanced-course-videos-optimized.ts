/**
 * OPTIMIZED Enhanced Course Video Generation Script
 * Uses NeoVerse, AvatarForcing, and VEO3 to create realistic robot training videos
 * 
 * OPTIMIZATIONS:
 * - Batch processing with controlled concurrency
 * - Skip already processed videos
 * - Reduced wait times
 * - Better error handling and retry logic
 * - Progress tracking
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

// Configuration
const CONCURRENT_VIDEOS = 3 // Process 3 videos at a time
const RETRY_ATTEMPTS = 2
const RETRY_DELAY = 5000 // 5 seconds
const BATCH_DELAY = 2000 // 2 seconds between batches (reduced from 5)

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
  courseSlug: string,
  retryCount = 0
): Promise<string | null> {
  try {
    // Determine template type
    const templateType = determineTemplateType(lesson.title, lesson.content)
    
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
        instructorAvatar: template.defaultConfig.instructorAvatar ? {
          ...template.defaultConfig.instructorAvatar,
          name: template.defaultConfig.instructorAvatar.name || 'Instructor'
        } : undefined
      }
    )

    // Update prompt with lesson-specific content
    const enhancedPrompt = `${basePrompt}\n\nLesson Content: ${lesson.content.substring(0, 1000)}`

    // Generate video with custom prompt
    const result = await enhancedVideoService.generateVideo(config, enhancedPrompt)

    if (result.success && result.taskId && result.videoId) {
      console.log(`✅ Video generation started for: ${lesson.title}`)
      console.log(`📝 Task ID: ${result.taskId}`)
      
      // Don't wait for completion - let it process in background
      // The polling will happen separately
      return result.taskId
    } else {
      throw new Error(result.error || 'Video generation failed')
    }
  } catch (error: any) {
    if (retryCount < RETRY_ATTEMPTS) {
      console.log(`⚠️  Retry ${retryCount + 1}/${RETRY_ATTEMPTS} for: ${lesson.title}`)
      await new Promise(resolve => setTimeout(resolve, RETRY_DELAY))
      return generateVideoForLesson(lesson, robotType, courseSlug, retryCount + 1)
    }
    console.error(`❌ Failed after ${RETRY_ATTEMPTS} attempts: ${lesson.title}`, error?.message || error)
    return null
  }
}

// Process videos in batches with concurrency control
async function processBatch(
  lessons: any[],
  robotType: 'unitree-g1' | 'kabuki2' | 'generic',
  courseSlug: string,
  batchNum: number,
  totalBatches: number
): Promise<Array<{ lessonId: string; lessonTitle: string; success: boolean; taskId?: string }>> {
  console.log(`\n📦 Processing batch ${batchNum}/${totalBatches} (${lessons.length} lessons)`)
  
  const results = await Promise.allSettled(
    lessons.map(lesson => 
      generateVideoForLesson(lesson, robotType, courseSlug)
        .then(taskId => ({
          lessonId: lesson.id,
          lessonTitle: lesson.title,
          success: !!taskId,
          taskId: taskId || undefined
        }))
    )
  )

  return results.map((result, index) => {
    if (result.status === 'fulfilled') {
      return result.value
    } else {
      return {
        lessonId: lessons[index].id,
        lessonTitle: lessons[index].title,
        success: false
      }
    }
  })
}

async function main() {
  try {
    console.log('🚀 Starting OPTIMIZED Enhanced Course Video Generation')
    console.log('📦 Using: NeoVerse + AvatarForcing + VEO3')
    console.log(`⚡ Concurrency: ${CONCURRENT_VIDEOS} videos at a time`)
    console.log(`🔄 Retry attempts: ${RETRY_ATTEMPTS}`)
    console.log('')

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

    const allResults: Array<{
      courseSlug: string
      lessonId: string
      lessonTitle: string
      success: boolean
      taskId?: string
    }> = []

    let totalLessons = 0
    let skippedLessons = 0

    // Process each course
    for (const course of courses) {
      const robotType = courseRobotMapping[course.slug] || 'generic'
      const allLessons = course.modules.flatMap(module => 
        module.lessons.map(lesson => ({ ...lesson, moduleTitle: module.title }))
      )

      // Filter out lessons that already have videos
      const lessonsToProcess = allLessons.filter(lesson => !lesson.videoUrl)
      const skipped = allLessons.length - lessonsToProcess.length
      skippedLessons += skipped
      totalLessons += allLessons.length

      if (lessonsToProcess.length === 0) {
        console.log(`\n⏭️  Skipping ${course.title} - all videos already exist`)
        continue
      }

      console.log(`\n${'='.repeat(60)}`)
      console.log(`📚 Course: ${course.title}`)
      console.log(`🤖 Robot Type: ${robotType}`)
      console.log(`📦 Total lessons: ${allLessons.length}`)
      console.log(`✅ Already have videos: ${skipped}`)
      console.log(`🎬 To process: ${lessonsToProcess.length}`)
      console.log(`${'='.repeat(60)}`)

      // Process in batches
      const batches: any[][] = []
      for (let i = 0; i < lessonsToProcess.length; i += CONCURRENT_VIDEOS) {
        batches.push(lessonsToProcess.slice(i, i + CONCURRENT_VIDEOS))
      }

      for (let i = 0; i < batches.length; i++) {
        const batchResults = await processBatch(
          batches[i],
          robotType,
          course.slug,
          i + 1,
          batches.length
        )

        batchResults.forEach(result => {
          allResults.push({
            courseSlug: course.slug,
            ...result
          })
        })

        // Small delay between batches to avoid rate limiting
        if (i < batches.length - 1) {
          await new Promise(resolve => setTimeout(resolve, BATCH_DELAY))
        }
      }
    }

    // Summary
    console.log('\n' + '='.repeat(80))
    console.log('📊 OPTIMIZED GENERATION SUMMARY')
    console.log('='.repeat(80))

    const successful = allResults.filter(r => r.success).length
    const failed = allResults.filter(r => !r.success).length

    console.log(`📚 Total lessons: ${totalLessons}`)
    console.log(`⏭️  Skipped (already have videos): ${skippedLessons}`)
    console.log(`🎬 Processed: ${allResults.length}`)
    console.log(`✅ Successful: ${successful}/${allResults.length}`)
    console.log(`❌ Failed: ${failed}/${allResults.length}`)
    console.log(`⚡ Success rate: ${((successful / allResults.length) * 100).toFixed(1)}%`)

    // Group by course
    const byCourse: Record<string, typeof allResults> = {}
    allResults.forEach(r => {
      if (!byCourse[r.courseSlug]) {
        byCourse[r.courseSlug] = []
      }
      byCourse[r.courseSlug].push(r)
    })

    console.log('\n📚 Results by Course:')
    Object.entries(byCourse).forEach(([slug, courseResults]) => {
      const courseSuccess = courseResults.filter(r => r.success).length
      console.log(`\n  ${slug}:`)
      console.log(`    ✅ ${courseSuccess}/${courseResults.length} videos started`)
    })

    if (successful > 0) {
      console.log('\n✅ Successfully started video generation:')
      allResults.filter(r => r.success).slice(0, 10).forEach(r => {
        console.log(`   - ${r.lessonTitle} (${r.courseSlug})`)
        if (r.taskId) console.log(`     Task ID: ${r.taskId}`)
      })
      if (successful > 10) {
        console.log(`   ... and ${successful - 10} more`)
      }
    }

    if (failed > 0) {
      console.log('\n❌ Failed to generate videos:')
      allResults.filter(r => !r.success).forEach(r => {
        console.log(`   - ${r.lessonTitle} (${r.courseSlug})`)
      })
    }

    console.log('\n🎉 Optimized video generation completed!')
    console.log('💡 Videos are processing in the background')
    console.log('📊 Check database for progress updates')
    console.log('🚀 Setting new benchmarks in online education!')
  } catch (error: any) {
    console.error('❌ Fatal error:', error?.message || error)
    process.exit(1)
  } finally {
    await prisma.$disconnect()
  }
}

main()
