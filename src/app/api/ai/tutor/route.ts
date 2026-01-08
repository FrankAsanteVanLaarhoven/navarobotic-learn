import { NextRequest, NextResponse } from 'next/server'
import { geminiCompleteService } from '@/lib/ai/gemini-complete-service'

/**
 * AI Tutor API Route
 * Handles LLM requests for the robotics tutor
 */
export async function POST(request: NextRequest) {
  try {
    const body = await request.json()
    const { prompt, mode, history } = body

    if (!prompt || typeof prompt !== 'string') {
      return NextResponse.json(
        { error: 'Prompt is required' },
        { status: 400 }
      )
    }

    // Build system instruction based on mode
    let systemInstruction = `You are an expert AI Robotics Tutor specializing in:
- ROS2 (Robot Operating System 2) development
- Inverse Kinematics and Forward Kinematics
- Physics simulation and control systems
- Python and C++ robotics programming
- PID controllers, path planning, and sensor fusion
- Unitree G1, Boston Dynamics Atlas, and other humanoid robots

You help students learn robotics through clear explanations, code examples, and guided questions.`

    if (mode === 'socratic') {
      systemInstruction += '\n\nUse the Socratic method: Ask guiding questions to help students discover answers themselves. Encourage critical thinking.'
    } else if (mode === 'coding') {
      systemInstruction += '\n\nFocus on generating practical, working code examples. Include comments and best practices.'
    } else {
      systemInstruction += '\n\nProvide clear, detailed explanations with examples. Break down complex concepts into understandable parts.'
    }

    // Build conversation history
    const messages: string[] = []
    
    // Add recent history (last 5 messages for context)
    if (history && Array.isArray(history)) {
      const recentHistory = history.slice(-5)
      recentHistory.forEach((msg: any) => {
        if (msg.role === 'user') {
          messages.push(`User: ${msg.content}`)
        } else if (msg.role === 'assistant') {
          messages.push(`Assistant: ${msg.content}`)
        }
      })
    }

    // Build full prompt
    const fullPrompt = messages.length > 0
      ? `${messages.join('\n\n')}\n\nUser: ${prompt}\n\nAssistant:`
      : prompt

    // Generate response using Gemini
    const result = await geminiCompleteService.generateText(fullPrompt, {
      model: 'gemini-3-flash',
      temperature: mode === 'coding' ? 0.3 : mode === 'socratic' ? 0.7 : 0.6,
      maxTokens: 2048,
      systemInstruction
    })

    if (!result.success || !result.content) {
      return NextResponse.json(
        { 
          error: result.error || 'Failed to generate response',
          content: 'I apologize, but I encountered an error processing your request. Please try again.',
          type: 'explanation'
        },
        { status: 500 }
      )
    }

    // Detect if response contains code
    const hasCode = result.content.includes('```') || 
                   result.content.includes('def ') || 
                   result.content.includes('import ') ||
                   result.content.includes('#include')

    return NextResponse.json({
      content: result.content,
      type: hasCode ? 'code' : 'explanation',
      metadata: result.metadata
    })

  } catch (error: any) {
    console.error('AI Tutor API error:', error)
    return NextResponse.json(
      { 
        error: 'Internal server error',
        content: 'I encountered an error. Please try again later.',
        type: 'explanation'
      },
      { status: 500 }
    )
  }
}
