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
    <div className="fixed bottom-4 left-4 z-50 bg-black/60 backdrop-blur-md rounded-lg p-3 text-white text-xs font-mono">
      <div className="flex items-center gap-4">
        <div>
          <div className="text-white/50">FPS</div>
          <div className={`text-lg font-bold ${fps >= 55 ? 'text-green-400' : fps >= 30 ? 'text-yellow-400' : 'text-red-400'}`}>
            {fps}
          </div>
        </div>
        <div>
          <div className="text-white/50">Frame Time</div>
          <div className="text-lg font-bold text-cyan-400">{frameTime}ms</div>
        </div>
      </div>
    </div>
  )
}