import { NextResponse } from 'next/server'
import { getServerSession } from 'next-auth'
import { authOptions } from '@/lib/auth'
import { getCodeExecutionService } from '@/lib/code-execution'
import { z } from 'zod'

const executeSchema = z.object({
  code: z.string().min(1).max(10000),
  language: z.enum(['python', 'javascript', 'cpp', 'rust']),
  input: z.string().optional(),
  timeout: z.number().optional()
})

export async function POST(request: Request) {
  try {
    const session = await getServerSession(authOptions)

    if (!session) {
      return NextResponse.json(
        { error: 'Unauthorized' },
        { status: 401 }
      )
    }

    const body = await request.json()
    const validatedData = executeSchema.parse(body)

    const codeService = getCodeExecutionService()

    if (!codeService) {
      return NextResponse.json(
        { error: 'Code execution service not available' },
        { status: 503 }
      )
    }

    const result = await codeService.executeCode({
      code: validatedData.code,
      language: validatedData.language,
      input: validatedData.input,
      timeout: validatedData.timeout
    })

    return NextResponse.json(result)
  } catch (error: any) {
    if (error instanceof z.ZodError) {
      return NextResponse.json(
        { error: 'Invalid input', details: error.errors },
        { status: 400 }
      )
    }

    console.error('Code execution error:', error)
    return NextResponse.json(
      { error: 'Execution failed' },
      { status: 500 }
    )
  }
}
