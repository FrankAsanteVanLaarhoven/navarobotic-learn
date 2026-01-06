/**
 * Find a video link that has been generated
 */

import { config } from 'dotenv'
config()

import { PrismaClient } from '@prisma/client'

const prisma = new PrismaClient()

async function main() {
  try {
    console.log('🔍 Searching for generated videos...\n')

    // First, check for completed videos with URLs
    const allCompleted = await prisma.generatedVideo.findMany({
      where: {
        status: 'completed'
      },
      include: {
        lesson: {
          include: {
            module: {
              include: {
                course: {
                  select: {
                    title: true,
                    slug: true
                  }
                }
              }
            }
          }
        }
      },
      orderBy: { updatedAt: 'desc' },
      take: 10
    })

    const completedVideo = allCompleted.find(v => v.videoUrl && v.videoUrl.trim() !== '')

    if (completedVideo && completedVideo.videoUrl) {
      console.log('✅ Found completed video!\n')
      console.log('📹 Video Details:')
      console.log(`   Title: ${completedVideo.lesson?.title || 'Unknown'}`)
      console.log(`   Course: ${completedVideo.lesson?.module?.course?.title || 'Unknown'}`)
      console.log(`   Course Slug: ${completedVideo.lesson?.module?.course?.slug || 'unknown'}`)
      console.log('')
      console.log('🔗 Links:')
      console.log(`   Direct Video URL: ${completedVideo.videoUrl}`)
      console.log(`   Course Page: http://localhost:3000/courses/${completedVideo.lesson?.module?.course?.slug}`)
      console.log('')
      console.log('💡 To view:')
      console.log('   1. Start dev server: bun run dev')
      console.log('   2. Go to the Course Page link above')
      console.log('   3. Click on the lesson in the sidebar')
      console.log('   4. Click the "Video" tab')
      return
    }

    // Check for lessons with video URLs
    const lessonsWithVideos = await prisma.lesson.findMany({
      where: {},
      take: 50,
      orderBy: { updatedAt: 'desc' }
    })

    const lessonWithVideo = lessonsWithVideos.find(l => 
      l.videoUrl && 
      l.videoUrl.trim() !== '' &&
      !l.videoUrl.includes('sample') &&
      !l.videoUrl.includes('placeholder') &&
      !l.videoUrl.includes('example')
    )

    if (lessonWithVideo) {
      // Get full lesson data
      const fullLesson = await prisma.lesson.findUnique({
        where: { id: lessonWithVideo.id },
        include: {
          module: {
            include: {
              course: {
                select: {
                  title: true,
                  slug: true
                }
              }
            }
          }
        }
      })

      if (fullLesson && fullLesson.videoUrl) {
        console.log('✅ Found lesson with video URL!\n')
        console.log('📹 Lesson Details:')
        console.log(`   Title: ${fullLesson.title}`)
        console.log(`   Course: ${fullLesson.module.course.title}`)
        console.log(`   Course Slug: ${fullLesson.module.course.slug}`)
        console.log('')
        console.log('🔗 Links:')
        console.log(`   Direct Video URL: ${fullLesson.videoUrl}`)
        console.log(`   Course Page: http://localhost:3000/courses/${fullLesson.module.course.slug}`)
        console.log('')
        console.log('💡 To view:')
        console.log('   1. Start dev server: bun run dev')
        console.log(`   2. Go to: http://localhost:3000/courses/${fullLesson.module.course.slug}`)
        console.log(`   3. Click on: ${fullLesson.title}`)
        console.log('   4. Click the "Video" tab')
        return
      }
    }

    // Check for videos at high progress
    const highProgressVideo = await prisma.generatedVideo.findFirst({
      where: {
        progress: { gte: 95 }
      },
      include: {
        lesson: {
          include: {
            module: {
              include: {
                course: {
                  select: {
                    title: true,
                    slug: true
                  }
                }
              }
            }
          }
        }
      },
      orderBy: { progress: 'desc' }
    })

    if (highProgressVideo) {
      console.log('📊 Video at high progress (almost ready):\n')
      console.log('📹 Video Details:')
      console.log(`   Title: ${highProgressVideo.lesson?.title || 'Unknown'}`)
      console.log(`   Course: ${highProgressVideo.lesson?.module?.course?.title || 'Unknown'}`)
      console.log(`   Status: ${highProgressVideo.status}`)
      console.log(`   Progress: ${highProgressVideo.progress.toFixed(1)}%`)
      console.log('')
      console.log('🔗 Course Page:')
      console.log(`   http://localhost:3000/courses/${highProgressVideo.lesson?.module?.course?.slug}`)
      console.log('')
      console.log('⏳ This video is still processing. Check back in a few minutes!')
      console.log('   Or check status with: bun run check:videos')
      return
    }

    console.log('⏳ No videos ready yet.')
    console.log('   Videos are still being generated.')
    console.log('   Check status with: bun run check:videos')
    console.log('   Or monitor with: bun run monitor:videos')

  } catch (error: any) {
    console.error('❌ Error:', error?.message || error)
  } finally {
    await prisma.$disconnect()
  }
}

main()
