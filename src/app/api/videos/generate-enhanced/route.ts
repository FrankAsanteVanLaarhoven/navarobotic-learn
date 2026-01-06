/**
 * Enhanced Video Generation API Endpoint
 * Integrates NeoVerse, AvatarForcing, and VEO3
 */

import { NextResponse } from 'next/server'
import { db } from '@/lib/db'
import { enhancedVideoService } from '@/lib/video-generation/enhanced-video-service'
import { getRobotTemplate, createVideoConfigFromTemplate } from '@/lib/video-generation/robot-templates'
import type { VideoGenerationConfig } from '@/lib/video-generation/enhanced-video-service'

export async function POST(request: Request) {
  try {
    const body = await request.json()
    const {
      lessonId,
      robotType = 'unitree-g1',
      useNeoVerse = true,
      useAvatarForcing = true,
      useVEO3 = true,
      quality = 'high',
      duration = 30,
      resolution = '1080p',
      instructorAvatar,
      simulationEnvironment
    } = body

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

    // Determine template type from lesson
    const lessonTitleLower = lesson.title.toLowerCase()
    let templateType: 'introduction' | 'demonstration' | 'technical' | 'tutorial' = 'technical'
    
    if (lessonTitleLower.includes('introduction') || lessonTitleLower.includes('overview')) {
      templateType = 'introduction'
    } else if (lessonTitleLower.includes('demonstration') || lessonTitleLower.includes('demo')) {
      templateType = 'demonstration'
    } else if (lessonTitleLower.includes('tutorial') || lessonTitleLower.includes('step-by-step')) {
      templateType = 'tutorial'
    }

    // Get robot template and create config
    const template = getRobotTemplate(robotType)
    const baseConfig = createVideoConfigFromTemplate(
      robotType,
      lessonId,
      templateType,
      {
        useNeoVerse,
        useAvatarForcing,
        useVEO3,
        quality,
        duration,
        resolution: resolution as '720p' | '1080p' | '4K',
        instructorAvatar: instructorAvatar || template.defaultConfig.instructorAvatar,
        simulationEnvironment: simulationEnvironment || template.defaultConfig.simulationEnvironment
      }
    )

    console.log(`🎬 Generating enhanced video for lesson: ${lesson.title}`)
    console.log(`🤖 Robot: ${robotType}`)
    console.log(`📦 Features: NeoVerse=${useNeoVerse}, AvatarForcing=${useAvatarForcing}, VEO3=${useVEO3}`)

    // Generate video
    const result = await enhancedVideoService.generateVideo(baseConfig)

    if (!result.success) {
      return NextResponse.json(
        { 
          error: 'Failed to generate video',
          message: result.error 
        },
        { status: 500 }
      )
    }

    return NextResponse.json({
      success: true,
      taskId: result.taskId,
      videoId: result.videoId,
      status: 'processing',
      metadata: result.metadata,
      message: 'Enhanced video generation started. Poll /api/videos/status/[taskId] for updates.'
    })

  } catch (error: any) {
    console.error('Error generating enhanced video:', error)
    return NextResponse.json(
      { 
        error: 'Failed to generate video',
        message: error?.message || 'Unknown error'
      },
      { status: 500 }
    )
  }
}

/**
 * GET endpoint to check video generation status
 */
export async function GET(request: Request) {
  try {
    const { searchParams } = new URL(request.url)
    const taskId = searchParams.get('taskId')
    const videoId = searchParams.get('videoId')

    if (!taskId || !videoId) {
      return NextResponse.json(
        { error: 'Task ID and Video ID are required' },
        { status: 400 }
      )
    }

    const result = await enhancedVideoService.pollVideoStatus(taskId, videoId)

    return NextResponse.json(result)
  } catch (error: any) {
    console.error('Error checking video status:', error)
    return NextResponse.json(
      { 
        error: 'Failed to check video status',
        message: error?.message || 'Unknown error'
      },
      { status: 500 }
    )
  }
}
