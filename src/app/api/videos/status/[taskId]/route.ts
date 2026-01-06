import { NextResponse } from 'next/server'
import { db } from '@/lib/db'
import ZAI from 'z-ai-web-dev-sdk'

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

    // Initialize ZAI
    const zai = await ZAI.create()

    // Query task status
    const result = await zai.async.result.query(taskId)

    const response: any = {
      taskId: taskId,
      status: result.task_status,
      progress: result.task_status === 'PROCESSING' ? 50 : result.task_status === 'SUCCESS' ? 100 : 0
    }

    if (result.task_status === 'SUCCESS') {
      const videoUrl = 
        result.video_result?.[0]?.url || 
        result.video_url || 
        result.url || 
        result.video

      if (videoUrl) {
        response.videoUrl = videoUrl

        // Update GeneratedVideo record if it exists
        const generatedVideo = await db.generatedVideo.findFirst({
          where: {
            // Try to find by matching prompt or lesson
          }
        })

        if (generatedVideo) {
          await db.generatedVideo.update({
            where: { id: generatedVideo.id },
            data: {
              videoUrl: videoUrl,
              status: 'completed',
              progress: 100
            }
          })

          // Update lesson with video URL
          if (generatedVideo.lessonId) {
            await db.lesson.update({
              where: { id: generatedVideo.lessonId },
              data: { videoUrl: videoUrl }
            })
          }
        }
      }
    } else if (result.task_status === 'FAIL') {
      response.error = 'Video generation failed'
      
      // Update GeneratedVideo record
      const generatedVideo = await db.generatedVideo.findFirst({
        where: {}
      })

      if (generatedVideo) {
        await db.generatedVideo.update({
          where: { id: generatedVideo.id },
          data: {
            status: 'failed',
            errorMessage: 'Generation failed'
          }
        })
      }
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
