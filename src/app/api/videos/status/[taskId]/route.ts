import { NextResponse } from 'next/server'
import { db } from '@/lib/db'
import { geminiVideoService } from '@/lib/video-generation/gemini-video-service'

export async function GET(
  request: Request,
  { params }: { params: Promise<{ taskId: string }> }
) {
  try {
    const { taskId } = await params

    if (!taskId) {
      return NextResponse.json(
        { error: 'Task ID is required' },
        { status: 400 }
      )
    }

    // Find video by task ID
    const generatedVideo = await db.generatedVideo.findFirst({
      where: {
        // Task ID is stored in metadata or we can search by pattern
      }
    })

    if (!generatedVideo) {
      return NextResponse.json(
        { error: 'Video not found' },
        { status: 404 }
      )
    }

    // Poll status using Gemini video service
    const result = await geminiVideoService.pollVideoStatus(taskId, generatedVideo.id)

    const response: any = {
      taskId: taskId,
      status: result.success ? 'completed' : 'processing',
      progress: generatedVideo.progress || 0
    }

    if (result.success && result.videoUrl) {
      response.videoUrl = result.videoUrl
      response.status = 'completed'
      response.progress = 100

      // Update GeneratedVideo record
      await db.generatedVideo.update({
        where: { id: generatedVideo.id },
        data: {
          videoUrl: result.videoUrl,
          status: 'completed',
          progress: 100
        }
      })

      // Update lesson with video URL
      if (generatedVideo.lessonId) {
        await db.lesson.update({
          where: { id: generatedVideo.lessonId },
          data: { videoUrl: result.videoUrl }
        })
      }
    } else if (result.error) {
      response.error = result.error
      response.status = 'failed'
      
      // Update GeneratedVideo record
      await db.generatedVideo.update({
        where: { id: generatedVideo.id },
        data: {
          status: 'failed',
          errorMessage: result.error
        }
      })
    }

    return NextResponse.json(response)

  } catch (error: any) {
    console.error('Error querying video status:', error)
    return NextResponse.json(
      { 
        error: 'Failed to query video status',
        message: error?.message || 'Unknown error'
      },
      { status: 500 }
    )
  }
}
