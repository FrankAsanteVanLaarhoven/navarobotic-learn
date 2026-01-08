/**
 * Haptics Controller API
 * Provides force feedback using gamepad vibration or web-based haptics.
 * Supports collision feedback, grip resistance, and spectral haptics.
 */

export interface HapticEvent {
  type: 'collision' | 'resistance' | 'pulse' | 'continuous'
  duration: number
  intensity: number
  frequency?: number // For continuous buzzes
}

export class HapticController {
  private gamepadIndex: number | null = null
  private isSupported: boolean = false
  private activeEffect: any = null

  constructor() {
    this.checkSupport()
    this.setupGamepadEvents()
  }

  /**
   * Check for haptics support
   */
  private checkSupport(): void {
    // Check Gamepad Haptics API (Vibration Actuator)
    if (navigator.getGamepads) {
      const gamepads = navigator.getGamepads()
      // Gamepad API returns sparse array, need to find first non-null gamepad
      for (let i = 0; i < gamepads.length; i++) {
        const gamepad = gamepads[i]
        if (gamepad !== null) {
          this.gamepadIndex = gamepad.index
          
          // Check for haptic actuators
          this.isSupported = gamepad.vibrationActuator !== null
          break
        }
      }
    }

    // Check WebHaptics API (if browser extension/library is present)
    // @ts-ignore
    if (window.navigator.haptic) {
      this.isSupported = true
    }
  }

  /**
   * Set up gamepad event listeners
   */
  private setupGamepadEvents(): void {
    window.addEventListener('gamepadconnected', (event) => {
      this.gamepadIndex = event.gamepad.index
      console.log('Haptics controller connected:', event.gamepad.id)
    })

    window.addEventListener('gamepaddisconnected', (event) => {
      if (event.gamepad.index === this.gamepadIndex) {
        this.gamepadIndex = null
        this.isSupported = false
      }
    })
  }

  /**
   * Trigger a haptic effect
   */
  async trigger(event: HapticEvent): Promise<void> {
    if (!this.isSupported) {
      console.warn('Haptics not supported on this device')
      return
    }

    try {
      // Strategy 1: Gamepad Vibration
      if (this.gamepadIndex !== null) {
        const gamepad = navigator.getGamepads()[this.gamepadIndex]
        
        if (!gamepad) {
          console.warn('Gamepad not found at index', this.gamepadIndex)
          // Fallback to audio haptics
          this.playSpectralHaptics(event)
          return
        }
        
        if (gamepad.vibrationActuator) {
          const actuator = gamepad.vibrationActuator
          
          // Map event to actuator parameters
          const effectType = event.type === 'collision' ? 'heavy-click' 
                          : event.type === 'pulse' ? 'tick' 
                          : 'click'

          // Intensity mapping (0.0 to 1.0)
          const intensity = Math.min(1.0, Math.max(0.0, event.intensity))

          // Play effect
          await actuator.playEffect(effectType, {
            duration: event.duration,
            startDelay: 0,
            weakMagnitude: intensity * 0.5,
            strongMagnitude: intensity
          })
        } else if (gamepad.vibrationActuator) {
          // Fallback to simple vibration
          gamepad.vibrateActuator?.playEffect('click', { duration: 100, startDelay: 0, weakMagnitude: 0.5, strongMagnitude: 1.0 })
        }
      }

      // Strategy 2: Spectral Haptics (Audio)
      // This simulates touch using different sounds
      this.playSpectralHaptics(event)

      console.log(`Haptic event triggered: ${event.type}`)
    } catch (error) {
      console.error('Haptic trigger failed:', error)
    }
  }

  /**
   * Stop current haptic effect
   */
  async stop(): Promise<void> {
    if (this.gamepadIndex !== null) {
      const gamepad = navigator.getGamepads()[this.gamepadIndex]
      if (gamepad && gamepad.vibrationActuator) {
        await gamepad.vibrationActuator.reset()
      }
    }
    // Stop audio
    this.stopSpectralHaptics()
  }

  /**
   * Play spectral haptics (Sound-based)
   * Thud for metal, thump for plastic, buzz for electrical
   */
  private playSpectralHaptics(event: HapticEvent): void {
    // Create AudioContext
    const AudioContextClass = window.AudioContext || (window as any).webkitAudioContext
    if (!AudioContextClass) return
    
    const audioContext = new AudioContextClass()
    const oscillator = audioContext.createOscillator()
    const gainNode = audioContext.createGain()

    // Determine sound based on event type
    if (event.type === 'collision') {
      // Heavy thud
      oscillator.frequency.value = 50
      gainNode.gain.value = 1.0
    } else if (event.type === 'resistance') {
      // Low rumble
      oscillator.frequency.value = 30
      gainNode.gain.value = 0.5
    } else if (event.type === 'pulse') {
      // Quick tick
      oscillator.frequency.value = 800
      gainNode.gain.value = 0.2
    }

    // Connect nodes
    oscillator.connect(gainNode)
    gainNode.connect(audioContext.destination)

    // Play
    oscillator.start()
    
    // Stop after duration
    setTimeout(() => {
      oscillator.stop()
      gainNode.disconnect()
    }, event.duration)
  }

  private stopSpectralHaptics(): void {
    // Logic to stop any playing sounds
  }

  /**
   * Pre-defined effects
   */
  async collisionFeedback(intensity: 'light' | 'medium' | 'heavy'): Promise<void> {
    const intensityMap = { light: 0.3, medium: 0.6, heavy: 1.0 }
    await this.trigger({
      type: 'collision',
      duration: 200,
      intensity: intensityMap[intensity]
    })
  }

  async gripResistance(force: number): Promise<void> {
    // Scale resistance (0 to 1)
    const intensity = Math.min(1.0, force / 10)
    await this.trigger({
      type: 'continuous',
      duration: 100, // Continuous until released
      intensity: intensity,
      frequency: 50
    })
  }

  async successPulse(): Promise<void> {
    await this.trigger({
      type: 'pulse',
      duration: 300,
      intensity: 0.8
    })
  }
}

/**
 * Factory function
 */
export function createHapticController(): HapticController {
  return new HapticController()
}
