/**
 * AvatarForcing Integration
 * Real-time interactive head avatar generation for course videos
 */

export interface AvatarForcingConfig {
  enabled: boolean
  instructorName: string
  style: 'professional' | 'casual' | 'technical'
  voiceProfile?: string
  position: 'corner' | 'side-by-side' | 'fullscreen'
  reactivity: 'high' | 'medium' | 'low'
  latency: number // Target latency in ms (AvatarForcing achieves ~500ms)
}

export interface AvatarForcingPrompt {
  basePrompt: string
  instructorConfig: AvatarForcingConfig
  contentContext: string
  emotionalTone: 'neutral' | 'excited' | 'serious' | 'friendly'
}

/**
 * AvatarForcing Integration Service
 * Generates prompts and configurations for interactive instructor avatars
 */
export class AvatarForcingIntegration {
  private config: AvatarForcingConfig

  constructor(config: Partial<AvatarForcingConfig> = {}) {
    this.config = {
      enabled: config.enabled ?? true,
      instructorName: config.instructorName || 'Instructor',
      style: config.style || 'professional',
      voiceProfile: config.voiceProfile,
      position: config.position || 'corner',
      reactivity: config.reactivity || 'high',
      latency: config.latency || 500
    }
  }

  /**
   * Generate AvatarForcing prompt for video generation
   */
  generateAvatarPrompt(content: string, emotionalTone: AvatarForcingPrompt['emotionalTone'] = 'neutral'): string {
    if (!this.config.enabled) return content

    const styleDescriptions = {
      professional: 'professional instructor in business attire, clear and articulate speech, confident posture',
      casual: 'friendly instructor in casual clothing, approachable demeanor, engaging presentation style',
      technical: 'technical expert in lab coat, detailed explanations, precise gestures, analytical approach'
    }

    const toneDescriptions = {
      neutral: 'neutral expression, clear and informative',
      excited: 'enthusiastic expression, animated gestures, engaging energy',
      serious: 'focused expression, deliberate movements, authoritative presence',
      friendly: 'warm expression, welcoming gestures, approachable demeanor'
    }

    const positionDescriptions = {
      corner: 'instructor avatar appears in bottom-right corner, 25% of screen size',
      'side-by-side': 'instructor avatar on left side, robot demonstration on right, 50/50 split',
      fullscreen: 'instructor avatar fullscreen with robot demonstration in background or picture-in-picture'
    }

    return `${content}

AvatarForcing Interactive Instructor Avatar:
- Real-time interactive head avatar generation with ${this.config.latency}ms latency
- Instructor: ${this.config.instructorName}, ${styleDescriptions[this.config.style]}
- Emotional tone: ${toneDescriptions[emotionalTone]}
- Position: ${positionDescriptions[this.config.position]}
- Reactivity level: ${this.config.reactivity} - avatar responds to content with natural reactions

Avatar Features:
- Natural conversation style with expressive reactions
- Synchronized lip-sync with speech (using ${this.config.voiceProfile || 'default voice'})
- Realistic facial expressions matching content emotional tone
- Responsive head movements: nods for agreement, tilts for emphasis
- Eye contact with camera for engagement
- Gestures synchronized with speech points
- Real-time reactions to robot demonstrations: expressions of interest, surprise, approval
- Professional educational presentation style
- Low latency interaction feel (${this.config.latency}ms)

Avatar Technical Specifications:
- High-quality facial rendering with realistic skin textures
- Smooth motion with temporal consistency
- Natural blinking and micro-expressions
- Hair and clothing physics for realistic movement
- Professional lighting matching main video
- Seamless integration with video background`
  }

  /**
   * Generate audio script for TTS with avatar synchronization
   */
  generateAudioScript(content: string): string {
    // Add natural pauses and emphasis markers for better TTS
    let script = content
    
    // Add pauses after periods
    script = script.replace(/\. /g, '. [pause:0.5] ')
    
    // Add emphasis on key terms
    const keyTerms = ['robot', 'sensor', 'control', 'algorithm', 'important', 'note']
    keyTerms.forEach(term => {
      const regex = new RegExp(`\\b${term}\\b`, 'gi')
      script = script.replace(regex, `[emphasis]${term}[/emphasis]`)
    })

    return script
  }

  /**
   * Get voice profile configuration
   */
  getVoiceProfileConfig(): {
    provider: string
    voiceId: string
    settings: {
      stability: number
      similarityBoost: number
      style?: number
    }
  } {
    // Default voice profiles based on instructor style
    const voiceProfiles: Record<string, any> = {
      professional: {
        provider: 'elevenlabs',
        voiceId: this.config.voiceProfile || 'voice-adam',
        settings: {
          stability: 0.7,
          similarityBoost: 0.8,
          style: 0.5
        }
      },
      casual: {
        provider: 'elevenlabs',
        voiceId: this.config.voiceProfile || 'voice-bella',
        settings: {
          stability: 0.6,
          similarityBoost: 0.7,
          style: 0.6
        }
      },
      technical: {
        provider: 'elevenlabs',
        voiceId: this.config.voiceProfile || 'voice-adam',
        settings: {
          stability: 0.8,
          similarityBoost: 0.9,
          style: 0.4
        }
      }
    }

    return voiceProfiles[this.config.style] || voiceProfiles.professional
  }

  /**
   * Generate video prompt description for AvatarForcing features
   */
  generateVideoPromptDescription(): string {
    if (!this.config.enabled) return ''

    return `
AvatarForcing Interactive Avatar Features:
- Real-time head avatar generation with ${this.config.latency}ms latency
- Instructor: ${this.config.instructorName} (${this.config.style} style)
- Position: ${this.config.position}
- Natural conversation style with expressive reactions
- Synchronized lip-sync and facial expressions
- Responsive to content with appropriate reactions
- Professional educational presentation
`
  }

  /**
   * Update configuration
   */
  updateConfig(config: Partial<AvatarForcingConfig>): void {
    this.config = { ...this.config, ...config }
  }

  /**
   * Get current configuration
   */
  getConfig(): AvatarForcingConfig {
    return { ...this.config }
  }
}

// Export singleton instance
export const avatarForcingIntegration = new AvatarForcingIntegration()
