import { NextResponse } from 'next/server'
import { getServerSession } from 'next-auth'
import { authOptions } from '@/lib/auth'
import { getStorageService } from '@/lib/storage'

export async function POST(request: Request) {
  try {
    const session = await getServerSession(authOptions)

    if (!session) {
      return NextResponse.json(
        { error: 'Unauthorized' },
        { status: 401 }
      )
    }

    const formData = await request.formData()
    const file = formData.get('file') as File
    const folder = formData.get('folder') as string || 'uploads'

    if (!file) {
      return NextResponse.json(
        { error: 'No file provided' },
        { status: 400 }
      )
    }

    const storage = getStorageService()

    if (!storage) {
      return NextResponse.json(
        { error: 'Storage service not configured' },
        { status: 503 }
      )
    }

    const buffer = Buffer.from(await file.arrayBuffer())
    const key = `${folder}/${session.user.id}/${Date.now()}-${file.name}`

    const url = await storage.uploadFile(
      key,
      buffer,
      file.type,
      {
        userId: session.user.id,
        originalName: file.name
      }
    )

    return NextResponse.json({
      success: true,
      url,
      key
    })
  } catch (error: any) {
    console.error('Upload error:', error)
    return NextResponse.json(
      { error: 'Upload failed' },
      { status: 500 }
    )
  }
}
