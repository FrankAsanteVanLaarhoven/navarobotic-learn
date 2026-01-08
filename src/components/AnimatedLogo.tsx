'use client'

import { motion } from 'framer-motion'

interface AnimatedLogoProps {
  className?: string
  size?: 'sm' | 'md' | 'lg'
  showBackground?: boolean
}

export function AnimatedLogo({ className = '', size = 'md', showBackground = false }: AnimatedLogoProps) {
  const sizeClasses = {
    sm: 'h-6 text-base',
    md: 'h-8 text-2xl',
    lg: 'h-12 text-4xl'
  }

  const containerSizes = {
    sm: { width: 100, height: 30 },
    md: { width: 140, height: 40 },
    lg: { width: 200, height: 60 }
  }

  const currentSize = containerSizes[size]

  return (
    <div 
      className={`relative inline-flex items-center justify-center ${className}`}
      style={{ 
        perspective: '800px',
        perspectiveOrigin: 'center center',
        width: `${currentSize.width}px`,
        height: `${currentSize.height}px`
      }}
    >
      {/* Background Grid Pattern (optional) */}
      {showBackground && (
        <div 
          className="absolute inset-0 opacity-20 -z-10"
          style={{
            backgroundImage: `
              linear-gradient(rgba(59, 130, 246, 0.1) 1px, transparent 1px),
              linear-gradient(90deg, rgba(59, 130, 246, 0.1) 1px, transparent 1px)
            `,
            backgroundSize: '20px 20px'
          }}
        />
      )}

      {/* 3D Ring Container */}
      <div 
        className="absolute inset-0 flex items-center justify-center"
        style={{
          transformStyle: 'preserve-3d'
        }}
      >
        {/* Animated 3D Ring */}
        <motion.div
          className="absolute rounded-full border-2"
          style={{
            width: `${currentSize.width * 1.2}px`,
            height: `${currentSize.height * 0.6}px`,
            borderColor: 'rgba(20, 184, 166, 0.8)',
            borderWidth: '3px',
            boxShadow: `
              0 0 20px rgba(20, 184, 166, 0.6),
              0 0 40px rgba(6, 182, 212, 0.4),
              inset 0 0 20px rgba(20, 184, 166, 0.3)
            `,
            background: 'linear-gradient(135deg, rgba(20, 184, 166, 0.1) 0%, rgba(6, 182, 212, 0.1) 100%)',
            transformStyle: 'preserve-3d',
          }}
          animate={{
            rotateY: [0, 360],
            rotateX: [60, 60],
            opacity: [0.6, 1, 0.8, 1, 0.6],
          }}
          transition={{
            rotateY: { duration: 8, repeat: Infinity, ease: "linear" },
            opacity: { duration: 4, repeat: Infinity, ease: "easeInOut" }
          }}
        />

        {/* Secondary Ring for Depth */}
        <motion.div
          className="absolute rounded-full border"
          style={{
            width: `${currentSize.width * 1.15}px`,
            height: `${currentSize.height * 0.55}px`,
            borderColor: 'rgba(6, 182, 212, 0.5)',
            borderWidth: '2px',
            transformStyle: 'preserve-3d',
            filter: 'blur(1px)',
          }}
          animate={{
            rotateY: [180, 540],
            rotateX: [60, 60],
            opacity: [0.3, 0.7, 0.3],
          }}
          transition={{
            rotateY: { duration: 8, repeat: Infinity, ease: "linear" },
            opacity: { duration: 4, repeat: Infinity, ease: "easeInOut" }
          }}
        />
      </div>

      {/* ROVYN Text - Always on top */}
      <motion.div
        className={`relative z-20 font-bold tracking-tight ${sizeClasses[size]} flex items-center justify-center`}
        style={{
          color: '#e5e7eb',
          textShadow: '0 0 10px rgba(255, 255, 255, 0.3), 0 0 20px rgba(255, 255, 255, 0.1)',
          fontFamily: 'system-ui, -apple-system, sans-serif',
          fontWeight: 700,
          letterSpacing: '0.05em',
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center'
        }}
        initial={{ opacity: 0, y: 10 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.6, delay: 0.2 }}
      >
        ROVYN
      </motion.div>

      {/* Glowing Particles Effect */}
      <motion.div
        className="absolute inset-0 pointer-events-none"
        style={{
          background: `radial-gradient(circle at 50% 50%, rgba(20, 184, 166, 0.15) 0%, transparent 70%)`,
          zIndex: 1
        }}
        animate={{
          opacity: [0.3, 0.6, 0.3],
          scale: [1, 1.2, 1]
        }}
        transition={{
          duration: 3,
          repeat: Infinity,
          ease: "easeInOut"
        }}
      />
    </div>
  )
}
