'use client'

import { motion } from 'framer-motion'
import { useState, useEffect, ReactNode } from 'react'

interface MissionImpossibleTextProps {
  children: ReactNode
  className?: string
  variant?: 'heading' | 'body' | 'label' | 'small'
  withFrame?: boolean
  glowIntensity?: 'low' | 'medium' | 'high'
}

export function MissionImpossibleText({ 
  children, 
  className = '', 
  variant = 'body',
  withFrame = false,
  glowIntensity = 'medium'
}: MissionImpossibleTextProps) {
  const variantClasses = {
    heading: 'text-4xl sm:text-5xl lg:text-6xl font-bold',
    body: 'text-lg sm:text-xl',
    label: 'text-sm',
    small: 'text-xs'
  }

  const glowClasses = {
    low: 'text-shadow-mi-low',
    medium: 'text-shadow-mi-medium',
    high: 'text-shadow-mi-high'
  }

  const textContent = typeof children === 'string' ? children : String(children)
  const [hasGradient, setHasGradient] = useState(false)

  useEffect(() => {
    const checkGradient = () => {
      setHasGradient(localStorage.getItem('mission-use-gradient') === 'true')
    }
    checkGradient()
    window.addEventListener('mission-gradient-updated', checkGradient)
    return () => window.removeEventListener('mission-gradient-updated', checkGradient)
  }, [])

  return (
    <motion.div
      className={`mission-impossible-text ${variantClasses[variant]} ${glowClasses[glowIntensity]} ${className}`}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 0.5 }}
      data-text={textContent}
      data-has-gradient={hasGradient}
    >
      {withFrame && (
        <div className="mission-frame">
          <div className="frame-corner frame-corner-tl"></div>
          <div className="frame-corner frame-corner-tr"></div>
          <div className="frame-corner frame-corner-bl"></div>
          <div className="frame-corner frame-corner-br"></div>
          <div className="frame-line frame-line-top"></div>
          <div className="frame-line frame-line-bottom"></div>
          <div className="frame-line frame-line-left"></div>
          <div className="frame-line frame-line-right"></div>
        </div>
      )}
      <div className="mission-text-content">
        {children}
      </div>
    </motion.div>
  )
}
