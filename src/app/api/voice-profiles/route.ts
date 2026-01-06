import { NextResponse } from 'next/server'
import { db } from '@/lib/db'

export async function GET() {
  try {
    const voiceProfiles = await db.voiceProfile.findMany({
      where: {
        isActive: true
      },
      orderBy: {
        displayName: 'asc'
      }
    })

    return NextResponse.json(voiceProfiles)
  } catch (error) {
    console.error('Error fetching voice profiles:', error)
    return NextResponse.json(
      { error: 'Failed to fetch voice profiles' },
      { status: 500 }
    )
  }
}
