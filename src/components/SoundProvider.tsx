'use client'

import { useEffect } from 'react'
import { getSoundManager } from '@/lib/sounds'

/**
 * Global Sound Provider
 * Initializes sound system and adds sound effects to all interactive elements
 */
export function SoundProvider({ children }: { children: React.ReactNode }) {
  useEffect(() => {
    const soundManager = getSoundManager()
    
    // Initialize sound system on user interaction (browser audio policy)
    const initSound = () => {
      if (soundManager.isEnabled()) {
        // Play a subtle initialization sound
        soundManager.playBeep(600, 0.05)
      }
    }

    // Add global click sound to all buttons and interactive elements
    const handleGlobalClick = (e: MouseEvent) => {
      const target = e.target as HTMLElement
      
      // Check if target is an Element and has closest method
      if (!target || typeof target.closest !== 'function') return
      
      // Check if it's an interactive element
      if (
        target.tagName === 'BUTTON' ||
        target.closest('button') ||
        target.tagName === 'A' ||
        target.closest('a') ||
        target.getAttribute('role') === 'button' ||
        target.closest('[role="button"]')
      ) {
        // Only play if not already handled by component
        if (!target.closest('[data-sound-handled]')) {
          soundManager.playClick()
        }
      }
    }

    // Add global hover sound
    const handleGlobalHover = (e: MouseEvent) => {
      const target = e.target as HTMLElement
      
      // Check if target is an Element and has closest method
      if (!target || typeof target.closest !== 'function') return
      
      if (
        target.tagName === 'BUTTON' ||
        target.closest('button') ||
        target.tagName === 'A' ||
        target.closest('a') ||
        target.getAttribute('role') === 'button' ||
        target.closest('[role="button"]')
      ) {
        if (!target.closest('[data-sound-handled]')) {
          soundManager.playHover()
        }
      }
    }

    // Initialize on first user interaction
    document.addEventListener('click', initSound, { once: true })
    document.addEventListener('keydown', initSound, { once: true })

    // Add global listeners
    document.addEventListener('click', handleGlobalClick)
    document.addEventListener('mouseenter', handleGlobalHover, true)

    return () => {
      document.removeEventListener('click', handleGlobalClick)
      document.removeEventListener('mouseenter', handleGlobalHover, true)
    }
  }, [])

  return <>{children}</>
}
