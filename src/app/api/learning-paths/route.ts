import { NextResponse } from 'next/server'
import { db } from '@/lib/db'

export async function GET() {
  try {
    const learningPaths = await db.learningPath.findMany({
      include: {
        courses: {
          orderBy: {
            order: 'asc'
          }
        }
      },
      orderBy: {
        order: 'asc'
      }
    })

    return NextResponse.json(learningPaths)
  } catch (error) {
    console.error('Error fetching learning paths:', error)
    return NextResponse.json(
      { error: 'Failed to fetch learning paths' },
      { status: 500 }
    )
  }
}
