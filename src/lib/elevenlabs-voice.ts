/**
 * ElevenLabs Voice Integration
 * Provides human-realistic voice synthesis
 */

export interface ElevenLabsVoice {
  voice_id: string
  name: string
  category: string
  description: string
  gender: 'male' | 'female'
  accent: 'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'
  age: 'young' | 'middle' | 'mature'
  style: string[]
}

// Pre-configured ElevenLabs voices (best human-realistic voices)
export const ELEVENLABS_VOICES: ElevenLabsVoice[] = [
  // Male Voices
  {
    voice_id: 'pNInz6obpgDQGcFmaJgB', // Adam - Deep, warm male voice
    name: 'Adam',
    category: 'male',
    description: 'Deep, warm male voice with US accent',
    gender: 'male',
    accent: 'us',
    age: 'mature',
    style: ['professional', 'friendly', 'authoritative'],
  },
  {
    voice_id: 'EXAVITQu4vr4xnSDxMaL', // Bella - British male
    name: 'Bella',
    category: 'male',
    description: 'British male voice, clear and articulate',
    gender: 'male',
    accent: 'uk',
    age: 'middle',
    style: ['professional', 'clear', 'articulate'],
  },
  {
    voice_id: 'ErXwobaYiN019PkySvjV', // Antoni - Australian male
    name: 'Antoni',
    category: 'male',
    description: 'Australian male voice, friendly and casual',
    gender: 'male',
    accent: 'australia',
    age: 'young',
    style: ['friendly', 'casual', 'energetic'],
  },
  // Female Voices
  {
    voice_id: '21m00Tcm4TlvDq8ikWAM', // Rachel - Professional female US
    name: 'Rachel',
    category: 'female',
    description: 'Professional female voice with US accent',
    gender: 'female',
    accent: 'us',
    age: 'middle',
    style: ['professional', 'clear', 'warm'],
  },
  {
    voice_id: 'AZnzlk1XvdvUeBnXmlld', // Domi - British female
    name: 'Domi',
    category: 'female',
    description: 'British female voice, elegant and refined',
    gender: 'female',
    accent: 'uk',
    age: 'mature',
    style: ['elegant', 'refined', 'professional'],
  },
  {
    voice_id: 'ThT5KcBeYPX3keUQqHPh', // Dorothy - Australian female
    name: 'Dorothy',
    category: 'female',
    description: 'Australian female voice, friendly and approachable',
    gender: 'female',
    accent: 'australia',
    age: 'young',
    style: ['friendly', 'approachable', 'energetic'],
  },
]

export interface VoiceSettings {
  provider: 'elevenlabs' | 'browser'
  voiceId?: string
  gender?: 'male' | 'female'
  accent?: 'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'
  stability?: number // 0-1
  similarityBoost?: number // 0-1
  style?: number // 0-1
  useSpeakerBoost?: boolean
}

export class ElevenLabsVoiceService {
  private apiKey: string | null = null
  private baseUrl = 'https://api.elevenlabs.io/v1'

  constructor(apiKey?: string) {
    this.apiKey = apiKey || process.env.NEXT_PUBLIC_ELEVENLABS_API_KEY || null
  }

  async synthesize(text: string, voiceId: string, settings?: Partial<VoiceSettings>): Promise<ArrayBuffer> {
    if (!this.apiKey) {
      throw new Error('ElevenLabs API key not configured')
    }

    const response = await fetch(`${this.baseUrl}/text-to-speech/${voiceId}`, {
      method: 'POST',
      headers: {
        'Accept': 'audio/mpeg',
        'Content-Type': 'application/json',
        'xi-api-key': this.apiKey,
      },
      body: JSON.stringify({
        text,
        model_id: 'eleven_multilingual_v2', // Best quality model
        voice_settings: {
          stability: settings?.stability ?? 0.5,
          similarity_boost: settings?.similarityBoost ?? 0.75,
          style: settings?.style ?? 0.0,
          use_speaker_boost: settings?.useSpeakerBoost ?? true,
        },
      }),
    })

    if (!response.ok) {
      const error = await response.text()
      throw new Error(`ElevenLabs API error: ${error}`)
    }

    return await response.arrayBuffer()
  }

  async getVoices(): Promise<ElevenLabsVoice[]> {
    if (!this.apiKey) {
      // Return default voices if API key not configured
      return ELEVENLABS_VOICES
    }

    try {
      const response = await fetch(`${this.baseUrl}/voices`, {
        headers: {
          'xi-api-key': this.apiKey,
        },
      })

      if (!response.ok) {
        return ELEVENLABS_VOICES
      }

      const data = await response.json()
      return data.voices || ELEVENLABS_VOICES
    } catch (error) {
      console.warn('Failed to fetch ElevenLabs voices, using defaults:', error)
      return ELEVENLABS_VOICES
    }
  }

  async playAudio(audioBuffer: ArrayBuffer): Promise<void> {
    const audioContext = new (window.AudioContext || (window as any).webkitAudioContext)()
    const audioData = await audioContext.decodeAudioData(audioBuffer)
    const source = audioContext.createBufferSource()
    source.buffer = audioData
    source.connect(audioContext.destination)
    source.start(0)
  }

  isConfigured(): boolean {
    return !!this.apiKey
  }
}

// Browser-native voice selection
export function getBrowserVoices(): SpeechSynthesisVoice[] {
  if (typeof window === 'undefined' || !('speechSynthesis' in window)) {
    return []
  }

  return window.speechSynthesis.getVoices()
}

export function filterBrowserVoices(
  voices: SpeechSynthesisVoice[],
  gender?: 'male' | 'female',
  accent?: 'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'
): SpeechSynthesisVoice[] {
  return voices.filter((voice) => {
    const name = voice.name.toLowerCase()
    const lang = voice.lang.toLowerCase()

    // Gender filter (approximate based on voice name)
    if (gender) {
      const maleNames = ['male', 'man', 'david', 'john', 'james', 'daniel', 'michael']
      const femaleNames = ['female', 'woman', 'samantha', 'karen', 'susan', 'linda', 'emily', 'zira', 'hazel']
      
      if (gender === 'male' && !maleNames.some(n => name.includes(n))) {
        if (femaleNames.some(n => name.includes(n))) return false
      }
      if (gender === 'female' && !femaleNames.some(n => name.includes(n))) {
        if (maleNames.some(n => name.includes(n))) return false
      }
    }

    // Accent filter
    if (accent) {
      const accentMap: Record<string, string[]> = {
        us: ['en-us', 'en-us', 'united states'],
        uk: ['en-gb', 'en-uk', 'united kingdom', 'british'],
        australia: ['en-au', 'australia', 'australian'],
        canada: ['en-ca', 'canada', 'canadian'],
        irish: ['en-ie', 'ireland', 'irish'],
        scottish: ['en-scotland', 'scotland', 'scottish'],
      }

      const patterns = accentMap[accent] || []
      if (!patterns.some(p => lang.includes(p) || name.includes(p))) {
        return false
      }
    }

    return true
  })
}

export function getBestBrowserVoice(
  gender?: 'male' | 'female',
  accent?: 'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'
): SpeechSynthesisVoice | null {
  const voices = getBrowserVoices()
  const filtered = filterBrowserVoices(voices, gender, accent)
  
  if (filtered.length === 0) {
    return voices[0] || null
  }

  // Prefer voices with better quality indicators
  const preferred = filtered.find(v => 
    v.name.includes('Enhanced') || 
    v.name.includes('Premium') ||
    v.name.includes('Neural')
  )

  return preferred || filtered[0]
}
