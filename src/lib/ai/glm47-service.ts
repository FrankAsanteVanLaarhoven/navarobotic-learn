/**
 * GLM-4.7 API Service
 * Integration with GLM-4.7 via Novita AI or Glama Gateway
 */

export interface GLM47Config {
  apiKey: string
  baseUrl?: string
  model?: string
}

export interface GLM47Response {
  success: boolean
  content?: string
  error?: string
}

/**
 * GLM-4.7 API Service
 * Access GLM-4.7 model via Novita AI or Glama Gateway
 */
export class GLM47Service {
  private apiKey: string | null = null
  private baseUrl: string
  private model: string

  constructor(config?: Partial<GLM47Config>) {
    this.apiKey = 
      config?.apiKey || 
      process.env.GLM_API_KEY || 
      process.env.GLM47_API_KEY || 
      process.env.NOVITA_API_KEY || 
      null
    
    this.baseUrl = config?.baseUrl || 
      process.env.NOVITA_BASE_URL || 
      'https://api.novita.ai'
    
    this.model = config?.model || 'zai-org/glm-4.7'
  }

  /**
   * Generate content using GLM-4.7
   */
  async generateContent(prompt: string, options?: {
    maxTokens?: number
    temperature?: number
    systemPrompt?: string
  }): Promise<GLM47Response> {
    try {
      if (!this.apiKey || this.apiKey === 'your_glm_api_key_here') {
        return {
          success: false,
          error: 'GLM-4.7 API key not configured. Please set GLM_API_KEY or NOVITA_API_KEY in .env file'
        }
      }

      const url = `${this.baseUrl}/openai/v1/chat/completions`
      
      const response = await fetch(url, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          model: this.model,
          messages: [
            ...(options?.systemPrompt ? [{
              role: 'system',
              content: options.systemPrompt
            }] : []),
            {
              role: 'user',
              content: prompt
            }
          ],
          max_tokens: options?.maxTokens || 2000,
          temperature: options?.temperature || 0.7
        })
      })

      if (!response.ok) {
        const error = await response.text()
        return {
          success: false,
          error: `API request failed: ${error}`
        }
      }

      const data = await response.json()
      
      return {
        success: true,
        content: data.choices?.[0]?.message?.content || data.content || ''
      }
    } catch (error: any) {
      console.error('Error calling GLM-4.7 API:', error)
      return {
        success: false,
        error: error?.message || 'Unknown error'
      }
    }
  }

  /**
   * Generate enhanced video prompt using GLM-4.7
   */
  async enhanceVideoPrompt(basePrompt: string, context: {
    lessonTitle: string
    lessonContent: string
    robotType?: string
  }): Promise<string> {
    const systemPrompt = `You are an expert in creating world-class educational video prompts for robotics courses. 
Your prompts should be comprehensive, detailed, and designed to create videos that are better than Coursera, Udemy, Teachable, and The Construct.

Include:
- Step-by-step walkthroughs
- Interactive coding workshops
- Comprehensive explanations (WHAT, WHY, HOW, WHEN, WHERE, WHO)
- Real AI voice narration
- Split-screen showing instructor and code workspace
- Live coding demonstrations
- Robot simulation integration`

    const enhancementPrompt = `Enhance this video prompt for a robotics course lesson:

Lesson Title: ${context.lessonTitle}
Lesson Content: ${context.lessonContent.substring(0, 500)}
Robot Type: ${context.robotType || 'generic'}

Base Prompt:
${basePrompt}

Create an enhanced, world-class video prompt that includes:
1. Professional instructor introduction
2. Step-by-step content delivery
3. Interactive coding workshop
4. Comprehensive explanations
5. Real-world robotics examples
6. Summary and next steps

Make it detailed, engaging, and production-ready.`

    const result = await this.generateContent(enhancementPrompt, {
      systemPrompt,
      maxTokens: 3000,
      temperature: 0.7
    })

    if (result.success && result.content) {
      return result.content
    }

    // Fallback to base prompt if enhancement fails
    return basePrompt
  }
}

// Export singleton instance
export const glm47Service = new GLM47Service()
