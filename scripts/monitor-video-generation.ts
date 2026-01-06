/**
 * Monitor Video Generation Progress
 * Checks database for video generation status and provides updates
 */

import { config } from 'dotenv'
config()

import { PrismaClient } from '@prisma/client'

const prisma = new PrismaClient()

async function monitorProgress() {
  try {
    console.log('📊 Video Generation Progress Monitor\n')
    console.log('Press Ctrl+C to stop monitoring\n')

    let lastCompleted = 0
    let lastProcessing = 0

    const checkProgress = async () => {
      const stats = await Promise.all([
        prisma.generatedVideo.count({ where: { status: 'completed' } }),
        prisma.generatedVideo.count({ where: { status: 'processing' } }),
        prisma.generatedVideo.count({ where: { status: 'failed' } }),
        prisma.generatedVideo.count(),
        prisma.lesson.count({ where: { videoUrl: { not: null } } }),
        prisma.lesson.count()
      ])

      const [completed, processing, failed, total, lessonsWithVideo, totalLessons] = stats

      // Calculate progress
      const progress = totalLessons > 0 ? (lessonsWithVideo / totalLessons) * 100 : 0
      const completedDelta = completed - lastCompleted
      const processingDelta = processing - lastProcessing

      // Clear previous line and print new status
      process.stdout.write('\r\x1b[K')
      process.stdout.write(
        `📹 Videos: ✅ ${completed} | ⏳ ${processing} | ❌ ${failed} | 📊 ${total} total | ` +
        `📚 Lessons: ${lessonsWithVideo}/${totalLessons} (${progress.toFixed(1)}%)`
      )

      if (completedDelta > 0) {
        console.log(`\n✅ ${completedDelta} new video(s) completed!`)
      }

      if (processingDelta !== 0) {
        console.log(`⏳ Processing: ${processing} video(s)`)
      }

      lastCompleted = completed
      lastProcessing = processing

      // Check if all videos are done
      if (processing === 0 && total > 0) {
        console.log('\n\n🎉 All videos have finished processing!')
        console.log(`✅ Completed: ${completed}`)
        console.log(`❌ Failed: ${failed}`)
        console.log(`📊 Total: ${total}`)
        process.exit(0)
      }
    }

    // Check every 10 seconds
    const interval = setInterval(checkProgress, 10000)
    await checkProgress() // Initial check

    // Handle graceful shutdown
    process.on('SIGINT', async () => {
      clearInterval(interval)
      console.log('\n\n📊 Final Status:')
      await checkProgress()
      await prisma.$disconnect()
      process.exit(0)
    })
  } catch (error: any) {
    console.error('❌ Error monitoring progress:', error?.message || error)
    await prisma.$disconnect()
    process.exit(1)
  }
}

monitorProgress()
