import { NextResponse } from 'next/server'
import { db } from '@/lib/db'

export async function GET() {
  try {
    const models = await db.videoGenerationModel.findMany({
      where: {
        isActive: true
      },
      orderBy: {
        name: 'asc'
      }
    })

    return NextResponse.json(models)
  } catch (error) {
    console.error('Error fetching video models:', error)
    return NextResponse.json(
      { error: 'Failed to fetch video models' },
      { status: 500 }
    )
  }
}
