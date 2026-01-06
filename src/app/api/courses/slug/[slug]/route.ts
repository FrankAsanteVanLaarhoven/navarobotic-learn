import { NextResponse } from 'next/server'
import { db } from '@/lib/db'

export async function GET(
  request: Request,
  { params }: { params: Promise<{ slug: string }> }
) {
  try {
    const { slug } = await params
    const course = await db.course.findUnique({
      where: {
        slug
      },
      include: {
        modules: {
          include: {
            lessons: {
              orderBy: {
                order: 'asc'
              }
            },
            orderBy: {
              order: 'asc'
            }
          }
        },
        learningPath: true
      }
    })

    if (!course) {
      return NextResponse.json(
        { error: 'Course not found' },
        { status: 404 }
      )
    }

    return NextResponse.json(course)
  } catch (error) {
    console.error('Error fetching course by slug:', error)
    return NextResponse.json(
      { error: 'Failed to fetch course' },
      { status: 500 }
    )
  }
}
