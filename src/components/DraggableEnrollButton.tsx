'use client'

import { useState, useEffect, useRef } from 'react'
import { motion, useMotionValue } from 'framer-motion'
import Link from 'next/link'
import { Button } from '@/components/ui/button'
import { GripVertical } from 'lucide-react'

export function DraggableEnrollButton() {
  const [isDragging, setIsDragging] = useState(false)
  const [isInitialized, setIsInitialized] = useState(false)
  const constraintsRef = useRef<HTMLDivElement>(null)
  const x = useMotionValue(0)
  const y = useMotionValue(0)

  // Initialize position
  useEffect(() => {
    const initializePosition = () => {
      if (!constraintsRef.current) return

      const container = constraintsRef.current
      const containerRect = container.getBoundingClientRect()
      
      // Load saved position from localStorage
      const savedPosition = localStorage.getItem('enroll-button-position')
      
      if (savedPosition) {
        try {
          const { x: savedX, y: savedY } = JSON.parse(savedPosition)
          // Validate position is within container bounds
          const maxX = containerRect.width - 320 // button width
          const maxY = containerRect.height - 80 // button height
          const validX = Math.max(0, Math.min(savedX, maxX))
          const validY = Math.max(0, Math.min(savedY, maxY))
          x.set(validX)
          y.set(validY)
        } catch (e) {
          console.warn('Failed to load saved position')
          // Default to center
          const defaultX = (containerRect.width / 2) - 160
          const defaultY = containerRect.height * 0.2
          x.set(defaultX)
          y.set(defaultY)
        }
      } else {
        // Default position: center horizontally, 20% from top
        const defaultX = (containerRect.width / 2) - 160
        const defaultY = containerRect.height * 0.2
        x.set(defaultX)
        y.set(defaultY)
      }
      
      setIsInitialized(true)
    }

    // Initialize on mount and window resize
    initializePosition()
    window.addEventListener('resize', initializePosition)
    return () => window.removeEventListener('resize', initializePosition)
  }, [x, y])

  // Save position to localStorage
  const handleDragEnd = () => {
    setIsDragging(false)
    const currentX = x.get()
    const currentY = y.get()
    localStorage.setItem('enroll-button-position', JSON.stringify({ x: currentX, y: currentY }))
  }

  if (!isInitialized) {
    return null // Don't render until position is initialized
  }

  return (
    <div ref={constraintsRef} className="absolute inset-0 pointer-events-none z-20 overflow-hidden">
      <motion.div
        drag
        dragMomentum={false}
        dragElastic={0.1}
        dragConstraints={constraintsRef}
        onDragStart={() => {
          setIsDragging(true)
        }}
        onDragEnd={() => {
          handleDragEnd()
          // Reset dragging flag after a short delay to allow click
          setTimeout(() => setIsDragging(false), 100)
        }}
        style={{
          x,
          y,
          cursor: isDragging ? 'grabbing' : 'grab',
        }}
        className="pointer-events-auto"
        whileDrag={{ scale: 1.05, zIndex: 50, opacity: 0.9 }}
        whileHover={{ scale: 1.02 }}
        transition={{ type: 'spring', stiffness: 300, damping: 30 }}
      >
        <Link 
          href="/auth?tab=register" 
          className="group block"
          onClick={(e) => {
            // Prevent navigation if we just finished dragging (within 200ms)
            if (isDragging) {
              e.preventDefault()
            }
          }}
        >
          <Button 
            size="lg" 
            className="enroll-cta-button text-white font-bold px-14 py-9 text-2xl rounded-lg border border-cyan-500/30 cursor-grab active:cursor-grabbing relative overflow-hidden shadow-2xl"
            style={{
              // Translucent background with backdrop blur
              backgroundColor: 'rgba(20, 184, 166, 0.7)',
              backdropFilter: 'blur(10px)',
              WebkitBackdropFilter: 'blur(10px)',
              boxShadow: isDragging 
                ? '0 0 30px rgba(20, 184, 166, 0.8), 0 0 60px rgba(20, 184, 166, 0.5), 0 10px 20px rgba(0, 0, 0, 0.3)' 
                : '0 0 20px rgba(20, 184, 166, 0.5), 0 0 40px rgba(20, 184, 166, 0.3), 0 8px 16px rgba(0, 0, 0, 0.2)',
              transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
              borderRadius: '0.5rem',
              minWidth: '320px',
              letterSpacing: '0.02em',
              // Floating effect
              transform: 'translateZ(0)',
            }}
          >
            {/* Drag handle indicator */}
            <div className="absolute top-2 right-2 opacity-40 group-hover:opacity-60 transition-opacity">
              <GripVertical className="w-4 h-4 text-white" />
            </div>

            {/* Animated background glow on hover */}
            <span 
              className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300 rounded-lg"
              style={{
                background: 'radial-gradient(circle at center, rgba(20, 184, 166, 0.4) 0%, transparent 70%)',
                filter: 'blur(20px)',
              }}
            />
            
            {/* Content */}
            <span className="relative z-10 font-bold whitespace-nowrap flex items-center gap-2">
              Enroll Now
            </span>
          </Button>
        </Link>
      </motion.div>
    </div>
  )
}
