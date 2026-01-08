'use client'

import { useEffect, useState, useRef } from 'react'

export function PerformanceMonitor() {
  const [fps, setFps] = useState(60)
  const [frameTime, setFrameTime] = useState(16.67)
  const frameCountRef = useRef(0)
  const lastTimeRef = useRef(performance.now())
  const animationFrameRef = useRef<number>()

  useEffect(() => {
    const measureFPS = () => {
      frameCountRef.current++
      const currentTime = performance.now()
      const elapsed = currentTime - lastTimeRef.current

      // Update every second
      if (elapsed >= 1000) {
        const currentFps = Math.round((frameCountRef.current * 1000) / elapsed)
        const avgFrameTime = elapsed / frameCountRef.current

        setFps(currentFps)
        setFrameTime(avgFrameTime.toFixed(2))

        frameCountRef.current = 0
        lastTimeRef.current = currentTime
      }

      animationFrameRef.current = requestAnimationFrame(measureFPS)
    }

    animationFrameRef.current = requestAnimationFrame(measureFPS)

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current)
      }
    }
  }, [])

  return (
    <div className="fixed bottom-4 left-4 z-50 glass border border-border/50 rounded-lg p-3 text-xs font-mono">
      <div className="flex items-center gap-4">
        <div>
          <div className="text-muted-foreground">FPS</div>
          <div className={`text-lg font-bold ${
            fps >= 55 ? 'text-primary' : fps >= 30 ? 'text-accent' : 'text-destructive'
          }`}>
            {fps}
          </div>
        </div>
        <div>
          <div className="text-muted-foreground">Frame Time</div>
          <div className="text-lg font-bold text-primary">{frameTime}ms</div>
        </div>
      </div>
    </div>
  )
}