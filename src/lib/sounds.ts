/**
 * Futuristic Robotic Sound System
 * Generates procedural sounds for UI interactions
 */

class SoundManager {
  private audioContext: AudioContext | null = null
  private enabled: boolean = true
  private volume: number = 0.3

  constructor() {
    if (typeof window !== 'undefined') {
      try {
        this.audioContext = new (window.AudioContext || (window as any).webkitAudioContext)()
      } catch (e) {
        console.warn('AudioContext not supported')
      }
    }
  }

  private createTone(
    frequency: number,
    duration: number,
    type: OscillatorType = 'sine',
    envelope: { attack?: number; decay?: number; sustain?: number; release?: number } = {}
  ): AudioBufferSourceNode | null {
    if (!this.audioContext || !this.enabled) return null

    const { attack = 0.01, decay = 0.1, sustain = 0.7, release = 0.2 } = envelope
    const oscillator = this.audioContext.createOscillator()
    const gainNode = this.audioContext.createGain()
    const now = this.audioContext.currentTime

    // Ensure duration is valid and release doesn't exceed duration
    const safeDuration = Math.max(duration, attack + decay + release)
    const releaseStart = Math.max(now + safeDuration - release, now + attack + decay)

    oscillator.type = type
    oscillator.frequency.setValueAtTime(frequency, now)

    // ADSR Envelope
    gainNode.gain.setValueAtTime(0, now)
    gainNode.gain.linearRampToValueAtTime(this.volume, now + attack)
    gainNode.gain.linearRampToValueAtTime(this.volume * sustain, now + attack + decay)
    gainNode.gain.setValueAtTime(this.volume * sustain, releaseStart)
    gainNode.gain.linearRampToValueAtTime(0, now + safeDuration)

    oscillator.connect(gainNode)
    gainNode.connect(this.audioContext.destination)

    oscillator.start(now)
    oscillator.stop(now + safeDuration)

    return oscillator
  }

  // Click sound - sharp, decisive
  playClick() {
    this.createTone(800, 0.05, 'square', { attack: 0.001, release: 0.04 })
    // Add a subtle high-frequency click
    setTimeout(() => {
      this.createTone(1200, 0.03, 'sine', { attack: 0.001, release: 0.02 })
    }, 10)
  }

  // Hover sound - subtle, anticipatory
  playHover() {
    this.createTone(600, 0.08, 'sine', { attack: 0.02, decay: 0.06, sustain: 0.3, release: 0.02 })
  }

  // Select sound - confirmation, positive
  playSelect() {
    // Ascending tone sequence
    const frequencies = [400, 500, 600]
    frequencies.forEach((freq, i) => {
      setTimeout(() => {
        this.createTone(freq, 0.1, 'sine', { attack: 0.01, release: 0.09 })
      }, i * 50)
    })
  }

  // Success sound - positive completion
  playSuccess() {
    const frequencies = [523.25, 659.25, 783.99] // C, E, G major chord
    frequencies.forEach((freq, i) => {
      setTimeout(() => {
        this.createTone(freq, 0.2, 'sine', { attack: 0.02, decay: 0.1, sustain: 0.5, release: 0.08 })
      }, i * 30)
    })
  }

  // Error sound - negative feedback
  playError() {
    this.createTone(200, 0.15, 'sawtooth', { attack: 0.01, decay: 0.14 })
  }

  // Navigation sound - movement between items
  playNavigate() {
    this.createTone(700, 0.06, 'sine', { attack: 0.01, release: 0.05 })
  }

  // Toggle sound - state change
  playToggle(on: boolean) {
    if (on) {
      this.createTone(600, 0.1, 'sine', { attack: 0.02, release: 0.08 })
    } else {
      this.createTone(400, 0.1, 'sine', { attack: 0.02, release: 0.08 })
    }
  }

  // Robotic beep - system notification
  playBeep(frequency: number = 800, duration: number = 0.1) {
    this.createTone(frequency, duration, 'square', { attack: 0.01, release: 0.09 })
  }

  setVolume(volume: number) {
    this.volume = Math.max(0, Math.min(1, volume))
  }

  setEnabled(enabled: boolean) {
    this.enabled = enabled
  }

  isEnabled(): boolean {
    return this.enabled
  }
}

// Singleton instance
let soundManagerInstance: SoundManager | null = null

export function getSoundManager(): SoundManager {
  if (!soundManagerInstance) {
    soundManagerInstance = new SoundManager()
  }
  return soundManagerInstance
}

// React hook for easy access
export function useSounds() {
  return getSoundManager()
}
