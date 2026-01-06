/**
 * Check which courses have real generated videos (not samples)
 */

import { config } from 'dotenv'
config()

import { PrismaClient } from '@prisma/client'

const prisma = new PrismaClient()

async function main() {
  try {
    console.log('🔍 Checking for REAL generated videos...\n')

    // Check lessons with video URLs
    const lessonsWithVideos = await prisma.lesson.findMany({
      where: {
        videoUrl: { not: null }
      },
      select: {
        id: true,
        title: true,
        videoUrl: true,
        module: {
          select: {
            course: {
              select: {
                title: true,
                slug: true
              }
            }
          }
        }
      },
      take: 100
    })

    // Check generated videos
    const generatedVideos = await prisma.generatedVideo.findMany({
      where: {
        status: { in: ['completed', 'processing'] }
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
      take: 100
    })

    console.log('='.repeat(80))
    console.log('📚 COURSES WITH VIDEO URLs')
    console.log('='.repeat(80))

    if (lessonsWithVideos.length === 0) {
      console.log('⏳ No lessons with video URLs yet.')
    } else {
      const byCourse: Record<string, any> = {}
      lessonsWithVideos.forEach(lesson => {
        const slug = lesson.module.course.slug
        if (!byCourse[slug]) {
          byCourse[slug] = {
            title: lesson.module.course.title,
            lessons: [],
            realVideos: 0,
            sampleVideos: 0
          }
        }
        const isSample = lesson.videoUrl?.includes('sample') || 
                        lesson.videoUrl?.includes('placeholder') || 
                        lesson.videoUrl?.includes('example')
        byCourse[slug].lessons.push({
          title: lesson.title,
          videoUrl: lesson.videoUrl,
          isSample
        })
        if (isSample) {
          byCourse[slug].sampleVideos++
        } else {
          byCourse[slug].realVideos++
        }
      })

      Object.entries(byCourse).forEach(([slug, data]) => {
        console.log(`\n📖 ${data.title}`)
        console.log(`   Course Slug: ${slug}`)
        console.log(`   Access URL: http://localhost:3000/courses/${slug}`)
        console.log(`   ✅ Real Videos: ${data.realVideos}`)
        console.log(`   ⚠️  Sample/Placeholder: ${data.sampleVideos}`)
        console.log(`   📊 Total: ${data.lessons.length} lessons`)
        
        if (data.realVideos > 0) {
          console.log(`\n   Real Videos:`)
          data.lessons.filter((l: any) => !l.isSample).slice(0, 5).forEach((l: any) => {
            console.log(`     ✅ ${l.title}`)
            console.log(`        URL: ${l.videoUrl?.substring(0, 80)}...`)
          })
        }
      })
    }

    console.log('\n' + '='.repeat(80))
    console.log('🎬 GENERATED VIDEOS STATUS')
    console.log('='.repeat(80))

    if (generatedVideos.length === 0) {
      console.log('⏳ No generated videos yet.')
    } else {
      const byCourse: Record<string, any> = {}
      generatedVideos.forEach(video => {
        const slug = video.lesson?.module?.course?.slug || 'unknown'
        if (!byCourse[slug]) {
          byCourse[slug] = {
            title: video.lesson?.module?.course?.title || 'Unknown',
            completed: 0,
            processing: 0,
            videos: []
          }
        }
        byCourse[slug].videos.push(video)
        if (video.status === 'completed') {
          byCourse[slug].completed++
        } else {
          byCourse[slug].processing++
        }
      })

      Object.entries(byCourse).forEach(([slug, data]) => {
        console.log(`\n📖 ${data.title}`)
        console.log(`   Course Slug: ${slug}`)
        console.log(`   Access URL: http://localhost:3000/courses/${slug}`)
        console.log(`   ✅ Completed: ${data.completed}`)
        console.log(`   ⏳ Processing: ${data.processing}`)
        console.log(`   📊 Total: ${data.videos.length}`)
      })
    }

    // Summary
    console.log('\n' + '='.repeat(80))
    console.log('📊 SUMMARY')
    console.log('='.repeat(80))
    
    const stats = await Promise.all([
      prisma.generatedVideo.count({ where: { status: 'completed' } }),
      prisma.generatedVideo.count({ where: { status: 'processing' } }),
      prisma.generatedVideo.count({ where: { status: 'failed' } }),
      prisma.lesson.count({ where: { videoUrl: { not: null } } }),
      prisma.lesson.count()
    ])

    const [completed, processing, failed, withVideo, totalLessons] = stats

    console.log(`\nVideos:`)
    console.log(`  ✅ Completed: ${completed}`)
    console.log(`  ⏳ Processing: ${processing}`)
    console.log(`  ❌ Failed: ${failed}`)
    console.log(`  📊 Total: ${completed + processing + failed}`)
    
    console.log(`\nLessons:`)
    console.log(`  📚 With Video URLs: ${withVideo}/${totalLessons}`)
    console.log(`  📈 Progress: ${((withVideo / totalLessons) * 100).toFixed(1)}%`)

    console.log('\n💡 Access Videos:')
    console.log('  1. Via Course Page: http://localhost:3000/courses/[course-slug]')
    console.log('  2. Via API: GET /api/courses/slug/[course-slug]')
    console.log('  3. Monitor Progress: bun run monitor:videos')

  } catch (error: any) {
    console.error('❌ Error:', error?.message || error)
  } finally {
    await prisma.$disconnect()
  }
}

main()
