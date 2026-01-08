'use client'

import { useEffect, useRef } from 'react'
import { getSoundManager } from '@/lib/sounds'

interface UseSoundEffectsOptions {
  onClick?: boolean
  onHover?: boolean
  onSelect?: boolean
  enabled?: boolean
}

export function useSoundEffects(options: UseSoundEffectsOptions = {}) {
  const {
    onClick = true,
    onHover = true,
    onSelect = true,
    enabled = true,
  } = options

  const soundManager = getSoundManager()
  const elementRef = useRef<HTMLElement>(null)

  useEffect(() => {
    if (!enabled || !elementRef.current) return

    const element = elementRef.current

    const handleClick = () => {
      if (onClick) {
        soundManager.playClick()
      }
    }

    const handleMouseEnter = () => {
      if (onHover) {
        soundManager.playHover()
      }
    }

    const handleChange = () => {
      if (onSelect) {
        soundManager.playSelect()
      }
    }

    element.addEventListener('click', handleClick)
    element.addEventListener('mouseenter', handleMouseEnter)
    element.addEventListener('change', handleChange)

    return () => {
      element.removeEventListener('click', handleClick)
      element.removeEventListener('mouseenter', handleMouseEnter)
      element.removeEventListener('change', handleChange)
    }
  }, [enabled, onClick, onHover, onSelect, soundManager])

  return elementRef
}
