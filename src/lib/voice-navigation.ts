/**
 * Voice Navigation System
 * Real-time voice-controlled platform navigation
 */

export interface VoiceCommand {
  action: 'navigate' | 'search' | 'open' | 'close' | 'scroll' | 'help' | 'back' | 'home'
  target?: string
  query?: string
  direction?: 'up' | 'down' | 'left' | 'right'
}

export class VoiceNavigationEngine {
  private recognition: SpeechRecognition | null = null
  private synthesis: SpeechSynthesis | null = null
  private isListening: boolean = false
  private isActive: boolean = false
  private mouseIdleTimeout: NodeJS.Timeout | null = null
  private lastMouseActivity: number = Date.now()
  private mouseIdleThreshold: number = 3000 // 3 seconds

  constructor() {
    if (typeof window !== 'undefined') {
      this.initializeSpeechRecognition()
      this.initializeSpeechSynthesis()
      this.setupMouseIdleDetection()
      this.loadVoiceSettings()
    }
  }

  private initializeSpeechRecognition() {
    const SpeechRecognition = 
      (window as any).SpeechRecognition || 
      (window as any).webkitSpeechRecognition

    if (!SpeechRecognition) {
      console.warn('Speech Recognition not supported')
      return
    }

    this.recognition = new SpeechRecognition()
    this.recognition.continuous = true
    this.recognition.interimResults = true
    this.recognition.lang = 'en-US'
    this.recognition.maxAlternatives = 1

    this.recognition.onstart = () => {
      this.isListening = true
    }

    this.recognition.onend = () => {
      this.isListening = false
      // Auto-restart if voice navigation is active
      if (this.isActive && this.mouseIdleTimeout === null) {
        setTimeout(() => this.startListening(), 100)
      }
    }

    this.recognition.onerror = (event: any) => {
      console.error('Speech recognition error:', event.error)
      this.isListening = false
    }
  }

  private initializeSpeechSynthesis() {
    if ('speechSynthesis' in window) {
      this.synthesis = window.speechSynthesis
    }
  }

  private setupMouseIdleDetection() {
    const handleMouseMove = () => {
      this.lastMouseActivity = Date.now()
      
      // Clear existing timeout
      if (this.mouseIdleTimeout) {
        clearTimeout(this.mouseIdleTimeout)
        this.mouseIdleTimeout = null
      }

      // If voice navigation is active, deactivate it
      if (this.isActive) {
        this.deactivate()
      }

      // Set new timeout to activate voice navigation
      this.mouseIdleTimeout = setTimeout(() => {
        if (Date.now() - this.lastMouseActivity >= this.mouseIdleThreshold) {
          this.activate()
        }
      }, this.mouseIdleThreshold)
    }

    const handleMouseClick = () => {
      this.lastMouseActivity = Date.now()
      if (this.isActive) {
        this.deactivate()
      }
    }

    window.addEventListener('mousemove', handleMouseMove, { passive: true })
    window.addEventListener('mousedown', handleMouseClick, { passive: true })
    window.addEventListener('scroll', handleMouseMove, { passive: true })
  }

  parseCommand(transcript: string): VoiceCommand | null {
    const lower = transcript.toLowerCase().trim()

    // Enhanced navigation commands with better pattern matching
    const navPatterns = [
      /^(go to|navigate to|open|show|take me to|switch to|visit|load)\s+(.+)/i,
      /^(let's go to|let's visit|let's open)\s+(.+)/i,
      /^(i want to|i need to|i'd like to)\s+(go to|visit|open|see)\s+(.+)/i,
    ]

    for (const pattern of navPatterns) {
      const match = lower.match(pattern)
      if (match) {
        const target = match[match.length - 1] || ''
        const normalized = this.normalizeTarget(target.trim())
        if (normalized) {
          return { action: 'navigate', target: normalized }
        }
      }
    }

    // Direct page names
    const pageMap: Record<string, string> = {
      'home': '/',
      'courses': '/catalog',
      'course catalog': '/catalog',
      'catalog': '/catalog',
      'learning paths': '/catalog#paths',
      'learning path': '/catalog#paths',
      'simulation': '/simulation',
      'spatial simulation': '/simulation',
      'ai video studio': '/ai-video/generate',
      'ai video': '/ai-video/generate',
      'video studio': '/ai-video/generate',
      'generate video': '/ai-video/generate',
      'video portal': '/video',
      'video': '/video',
      'student': '/student',
      'student portal': '/student',
      'student dashboard': '/student',
      'admin': '/admin',
      'admin portal': '/admin',
      'admin dashboard': '/admin',
      'provider': '/provider',
      'provider portal': '/provider',
      'instructor': '/provider',
      'dashboard': '/student',
      'profile': '/student',
      'settings': '/student',
      'auth': '/auth',
      'login': '/auth?tab=login',
      'register': '/auth?tab=register',
      'sign up': '/auth?tab=register',
      'sign in': '/auth?tab=login',
      'authentication': '/auth',
    }

    for (const [key, path] of Object.entries(pageMap)) {
      if (lower.includes(key)) {
        return { action: 'navigate', target: path }
      }
    }

    // Enhanced scroll commands
    const scrollPatterns = [
      /^(scroll|move|go)\s+(up|down|left|right)/i,
      /^(scroll|move)\s+(the page|page)\s+(up|down|left|right)/i,
      /^(move|go)\s+(up|down|left|right)/i,
      /^(scroll up|scroll down|scroll left|scroll right)/i,
    ]

    for (const pattern of scrollPatterns) {
      const match = lower.match(pattern)
      if (match) {
        const direction = match.find((m, i) => i > 0 && ['up', 'down', 'left', 'right'].includes(m?.toLowerCase()))
        if (direction) {
          return { 
            action: 'scroll', 
            direction: direction.toLowerCase() as 'up' | 'down' | 'left' | 'right'
          }
        }
      }
    }

    // Enhanced search commands
    const searchPatterns = [
      /^(search|find|look for|search for|find me)\s+(.+)/i,
      /^(i want to|i need to)\s+(search|find|look for)\s+(.+)/i,
    ]

    for (const pattern of searchPatterns) {
      const match = lower.match(pattern)
      if (match) {
        const query = match[match.length - 1] || ''
        if (query.trim()) {
          return { action: 'search', query: query.trim() }
        }
      }
    }

    // Help command
    if (lower.includes('help') || lower.includes('what can you do')) {
      return { action: 'help' }
    }

    // Back command
    if (lower.includes('go back') || lower.includes('back')) {
      return { action: 'back' }
    }

    // Home command
    if (lower.includes('home') || lower.includes('main page')) {
      return { action: 'home' }
    }

    return null
  }

  private normalizeTarget(target: string): string {
    const normalized = target.toLowerCase().trim()
    const mappings: Record<string, string> = {
      'home': '/',
      'courses': '/catalog',
      'course catalog': '/catalog',
      'catalog': '/catalog',
      'learning paths': '/catalog#paths',
      'learning path': '/catalog#paths',
      'simulation': '/simulation',
      'spatial simulation': '/simulation',
      'ai video': '/ai-video/generate',
      'ai video studio': '/ai-video/generate',
      'video studio': '/ai-video/generate',
      'video portal': '/video',
      'video': '/video',
      'student': '/student',
      'student portal': '/student',
      'admin': '/admin',
      'admin portal': '/admin',
      'provider': '/provider',
      'provider portal': '/provider',
      'instructor': '/provider',
      'dashboard': '/student',
      'profile': '/student',
      'settings': '/student',
      'auth': '/auth',
      'login': '/auth?tab=login',
      'register': '/auth?tab=register',
      'sign up': '/auth?tab=register',
      'sign in': '/auth?tab=login',
    }

    return mappings[normalized] || `/${normalized.replace(/\s+/g, '-')}`
  }

  private voiceSettings: {
    provider: 'elevenlabs' | 'browser'
    voiceId?: string
    gender?: 'male' | 'female'
    accent?: 'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'
    browserVoice?: SpeechSynthesisVoice
  } = {
    provider: 'browser',
    accent: 'us',
  }

  setVoiceSettings(settings: {
    provider?: 'elevenlabs' | 'browser'
    voiceId?: string
    gender?: 'male' | 'female'
    accent?: 'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'
    browserVoice?: SpeechSynthesisVoice
  }) {
    this.voiceSettings = { ...this.voiceSettings, ...settings }
    // Save to localStorage
    if (typeof window !== 'undefined') {
      localStorage.setItem('voice-navigation-settings', JSON.stringify(this.voiceSettings))
    }
  }

  getVoiceSettings() {
    return { ...this.voiceSettings }
  }

  loadVoiceSettings() {
    if (typeof window === 'undefined') return

    const saved = localStorage.getItem('voice-navigation-settings')
    if (saved) {
      try {
        const settings = JSON.parse(saved)
        this.voiceSettings = { ...this.voiceSettings, ...settings }
      } catch (e) {
        console.warn('Failed to load voice settings')
      }
    }
  }

  async speak(text: string, onEnd?: () => void) {
    // Load settings if not loaded
    if (!this.voiceSettings.accent) {
      this.loadVoiceSettings()
    }

    if (this.voiceSettings.provider === 'elevenlabs' && this.voiceSettings.voiceId) {
      // Use ElevenLabs if configured
      try {
        const { ElevenLabsVoiceService } = await import('./elevenlabs-voice')
        const service = new ElevenLabsVoiceService()
        
        if (service.isConfigured()) {
          const audioBuffer = await service.synthesize(
            text,
            this.voiceSettings.voiceId,
            {
              gender: this.voiceSettings.gender,
              accent: this.voiceSettings.accent,
            }
          )
          
          const audioContext = new (window.AudioContext || (window as any).webkitAudioContext)()
          const audioData = await audioContext.decodeAudioData(audioBuffer)
          const source = audioContext.createBufferSource()
          source.buffer = audioData
          source.connect(audioContext.destination)
          
          if (onEnd) {
            source.onended = onEnd
          }
          
          source.start(0)
          return
        }
      } catch (error) {
        console.warn('ElevenLabs synthesis failed, falling back to browser:', error)
      }
    }

    // Fallback to browser TTS
    if (!this.synthesis) return

    this.synthesis.cancel()

    const utterance = new SpeechSynthesisUtterance(text)
    utterance.rate = 1.0
    utterance.pitch = this.voiceSettings.gender === 'male' ? 0.9 : 1.1
    utterance.volume = 0.8
    
    // Set language based on accent
    const langMap: Record<string, string> = {
      us: 'en-US',
      uk: 'en-GB',
      australia: 'en-AU',
      canada: 'en-CA',
      irish: 'en-IE',
      scottish: 'en-SC',
    }
    utterance.lang = langMap[this.voiceSettings.accent || 'us'] || 'en-US'

    // Use selected browser voice if available
    if (this.voiceSettings.browserVoice) {
      utterance.voice = this.voiceSettings.browserVoice
    } else {
      // Try to find best matching voice
      const { getBestBrowserVoice } = await import('./elevenlabs-voice')
      const bestVoice = getBestBrowserVoice(
        this.voiceSettings.gender,
        this.voiceSettings.accent
      )
      if (bestVoice) {
        utterance.voice = bestVoice
      }
    }

    if (onEnd) {
      utterance.onend = onEnd
    }

    this.synthesis.speak(utterance)
  }

  startListening(onResult: (command: VoiceCommand | null, transcript: string) => void) {
    if (!this.recognition) return
    
    if (this.isListening) {
      // Update the callback if already listening
      this.recognition.onresult = (event: SpeechRecognitionEvent) => {
        const transcript = Array.from(event.results)
          .map((result) => result[0].transcript)
          .join(' ')
          .trim()

        if (event.results[event.results.length - 1].isFinal) {
          const command = this.parseCommand(transcript)
          onResult(command, transcript)
        }
      }
      return
    }

    this.recognition.onresult = (event: SpeechRecognitionEvent) => {
      const transcript = Array.from(event.results)
        .map((result) => result[0].transcript)
        .join(' ')
        .trim()

      if (event.results[event.results.length - 1].isFinal) {
        const command = this.parseCommand(transcript)
        onResult(command, transcript)
      }
    }

    try {
      this.recognition.start()
    } catch (error) {
      console.error('Failed to start speech recognition:', error)
      // If already started, just update callback
      if ((error as Error).message.includes('already started')) {
        this.isListening = true
      }
    }
  }

  stopListening() {
    if (this.recognition && this.isListening) {
      this.recognition.stop()
      this.isListening = false
    }
  }

  activate(onCommand?: (command: VoiceCommand | null, transcript: string) => void) {
    if (this.isActive || !this.recognition) return
    
    this.isActive = true
    this.speak('Voice navigation activated. How can I help you?', () => {
      // Start listening after greeting
      if (onCommand) {
        this.startListening(onCommand)
      } else {
        this.startListening(() => {})
      }
    })
  }

  deactivate() {
    if (!this.isActive) return

    this.isActive = false
    this.stopListening()
    this.speak('Voice navigation deactivated')
  }

  isVoiceActive(): boolean {
    return this.isActive
  }

  isCurrentlyListening(): boolean {
    return this.isListening
  }

  destroy() {
    this.stopListening()
    if (this.mouseIdleTimeout) {
      clearTimeout(this.mouseIdleTimeout)
    }
    if (this.synthesis) {
      this.synthesis.cancel()
    }
  }
}

// Singleton instance
let voiceEngine: VoiceNavigationEngine | null = null

export function getVoiceNavigationEngine(): VoiceNavigationEngine {
  if (!voiceEngine && typeof window !== 'undefined') {
    voiceEngine = new VoiceNavigationEngine()
  }
  return voiceEngine!
}
