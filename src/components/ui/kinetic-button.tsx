'use client'

import React, { useRef, useState } from 'react'
import { motion, useMotionValue, useSpring, useTransform } from 'framer-motion'
import { cn } from '@/lib/utils'
import { getSoundManager } from '@/lib/sounds'
import { Button, ButtonProps } from '@/components/ui/button'

interface KineticButtonProps extends ButtonProps {
  soundOnClick?: boolean
  soundOnHover?: boolean
  glowIntensity?: 'low' | 'medium' | 'high'
  kineticEffect?: boolean
}

export function KineticButton({
  children,
  className,
  soundOnClick = true,
  soundOnHover = true,
  glowIntensity = 'medium',
  kineticEffect = true,
  onClick,
  onMouseEnter,
  onMouseLeave,
  ...props
}: KineticButtonProps) {
  const [isHovered, setIsHovered] = useState(false)
  const [isPressed, setIsPressed] = useState(false)
  const ref = useRef<HTMLButtonElement>(null)
  const soundManager = getSoundManager()

  // Kinetic gesture tracking
  const x = useMotionValue(0)
  const y = useMotionValue(0)
  const rotateX = useSpring(useTransform(y, [-0.5, 0.5], [5, -5]), { stiffness: 300, damping: 30 })
  const rotateY = useSpring(useTransform(x, [-0.5, 0.5], [-5, 5]), { stiffness: 300, damping: 30 })

  const glowClasses = {
    low: 'hover:shadow-[0_0_10px_rgba(6,182,212,0.3)]',
    medium: 'hover:shadow-[0_0_20px_rgba(6,182,212,0.5),0_0_40px_rgba(6,182,212,0.3)]',
    high: 'hover:shadow-[0_0_30px_rgba(6,182,212,0.7),0_0_60px_rgba(6,182,212,0.5),0_0_90px_rgba(6,182,212,0.3)]',
  }

  const handleMouseMove = (e: React.MouseEvent<HTMLButtonElement>) => {
    if (!ref.current || !kineticEffect) return

    const rect = ref.current.getBoundingClientRect()
    const centerX = rect.left + rect.width / 2
    const centerY = rect.top + rect.height / 2

    const deltaX = (e.clientX - centerX) / rect.width
    const deltaY = (e.clientY - centerY) / rect.height

    x.set(deltaX)
    y.set(deltaY)
  }

  const handleMouseEnter = (e: React.MouseEvent<HTMLButtonElement>) => {
    setIsHovered(true)
    if (soundOnHover) {
      soundManager.playHover()
    }
    onMouseEnter?.(e)
  }

  const handleMouseLeave = (e: React.MouseEvent<HTMLButtonElement>) => {
    setIsHovered(false)
    x.set(0)
    y.set(0)
    onMouseLeave?.(e)
  }

  const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
    setIsPressed(true)
    if (soundOnClick) {
      soundManager.playClick()
    }
    setTimeout(() => setIsPressed(false), 150)
    onClick?.(e)
  }

  return (
    <motion.div
      style={{
        rotateX: kineticEffect ? rotateX : 0,
        rotateY: kineticEffect ? rotateY : 0,
        transformStyle: 'preserve-3d',
      }}
      className="perspective-1000"
    >
      <Button
        ref={ref}
        className={cn(
          'relative overflow-hidden transition-all duration-200',
          glowClasses[glowIntensity],
          isHovered && 'border-cyan-500/50',
          isPressed && 'scale-95',
          className
        )}
        onMouseMove={handleMouseMove}
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
        onClick={handleClick}
        {...props}
      >
        {/* Ephemeral glow effect */}
        {isHovered && (
          <motion.div
            className="absolute inset-0 bg-gradient-to-r from-cyan-500/20 via-blue-500/20 to-cyan-500/20"
            initial={{ opacity: 0, x: '-100%' }}
            animate={{ opacity: 1, x: '100%' }}
            exit={{ opacity: 0 }}
            transition={{
              duration: 0.6,
              repeat: Infinity,
              ease: 'linear',
            }}
            style={{
              background: 'linear-gradient(90deg, transparent, rgba(6,182,212,0.3), transparent)',
            }}
          />
        )}

        {/* Ripple effect on click */}
        {isPressed && (
          <motion.div
            className="absolute inset-0 bg-cyan-500/30 rounded-full"
            initial={{ scale: 0, opacity: 0.8 }}
            animate={{ scale: 2, opacity: 0 }}
            transition={{ duration: 0.4 }}
          />
        )}

        {/* Content */}
        <span className="relative z-10">{children}</span>

        {/* Edge glow */}
        <motion.div
          className="absolute inset-0 border border-cyan-500/0 rounded-md"
          animate={{
            borderColor: isHovered ? 'rgba(6,182,212,0.5)' : 'rgba(6,182,212,0)',
          }}
          transition={{ duration: 0.2 }}
        />
      </Button>
    </motion.div>
  )
}
