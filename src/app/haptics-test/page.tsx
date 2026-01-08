'use client'

import { useState, useEffect } from 'react'
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Slider } from '@/components/ui/slider'
import { createHapticController } from '@/lib/haptics/haptic-controller'
import {
  Vibrate, Zap, AlertCircle, CheckCircle,
  Gamepad2, Volume2, Settings
} from 'lucide-react'

export default function HapticsTestPage() {
  const [hapticController, setHapticController] = useState<ReturnType<typeof createHapticController> | null>(null)
  const [isSupported, setIsSupported] = useState(false)
  const [gamepadConnected, setGamepadConnected] = useState(false)
  const [intensity, setIntensity] = useState([0.5])
  const [duration, setDuration] = useState([200])
  const [lastEvent, setLastEvent] = useState<string>('')

  useEffect(() => {
    // Initialize haptic controller
    const controller = createHapticController()
    setHapticController(controller)

    // Check for gamepad support
    const checkGamepad = () => {
      const gamepads = navigator.getGamepads()
      const hasGamepad = Array.from(gamepads).some(gp => gp !== null)
      setGamepadConnected(hasGamepad)
      setIsSupported(hasGamepad || 'vibrate' in navigator)
    }

    checkGamepad()

    // Listen for gamepad connect/disconnect
    window.addEventListener('gamepadconnected', () => {
      setGamepadConnected(true)
      setIsSupported(true)
    })

    window.addEventListener('gamepaddisconnected', () => {
      const gamepads = navigator.getGamepads()
      const hasGamepad = Array.from(gamepads).some(gp => gp !== null)
      setGamepadConnected(hasGamepad)
      setIsSupported(hasGamepad || 'vibrate' in navigator)
    })

    // Poll for gamepad (some browsers don't fire events)
    const gamepadInterval = setInterval(checkGamepad, 1000)

    return () => {
      clearInterval(gamepadInterval)
    }
  }, [])

  const triggerHaptic = async (type: 'collision' | 'resistance' | 'pulse' | 'continuous') => {
    if (!hapticController) return

    try {
      await hapticController.trigger({
        type,
        duration: duration[0],
        intensity: intensity[0]
      })
      setLastEvent(`${type} - ${intensity[0] * 100}% intensity, ${duration[0]}ms`)
    } catch (error) {
      console.error('Haptic trigger failed:', error)
      setLastEvent('Error: ' + (error as Error).message)
    }
  }

  const triggerCollision = async (level: 'light' | 'medium' | 'heavy') => {
    if (!hapticController) return
    try {
      await hapticController.collisionFeedback(level)
      setLastEvent(`Collision: ${level}`)
    } catch (error) {
      console.error('Collision feedback failed:', error)
    }
  }

  const triggerGripResistance = async () => {
    if (!hapticController) return
    try {
      await hapticController.gripResistance(intensity[0] * 10)
      setLastEvent(`Grip Resistance: ${intensity[0] * 10}N`)
    } catch (error) {
      console.error('Grip resistance failed:', error)
    }
  }

  const triggerSuccess = async () => {
    if (!hapticController) return
    try {
      await hapticController.successPulse()
      setLastEvent('Success Pulse')
    } catch (error) {
      console.error('Success pulse failed:', error)
    }
  }

  return (
    <div className="min-h-screen p-6 bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900">
      <div className="max-w-6xl mx-auto space-y-6">
        {/* Header */}
        <div className="text-center mb-8">
          <h1 className="text-4xl font-bold text-white mb-2">Haptics Test Suite</h1>
          <p className="text-slate-400">Test force feedback and tactile integration</p>
        </div>

        {/* Status Card */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white flex items-center gap-2">
              <Gamepad2 className="w-5 h-5 text-blue-400" />
              Device Status
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-slate-400">Haptics Support</span>
              <Badge variant={isSupported ? 'default' : 'destructive'}>
                {isSupported ? (
                  <>
                    <CheckCircle className="w-3 h-3 mr-1" />
                    Supported
                  </>
                ) : (
                  <>
                    <AlertCircle className="w-3 h-3 mr-1" />
                    Not Supported
                  </>
                )}
              </Badge>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-slate-400">Gamepad Connected</span>
              <Badge variant={gamepadConnected ? 'default' : 'outline'}>
                {gamepadConnected ? (
                  <>
                    <CheckCircle className="w-3 h-3 mr-1" />
                    Connected
                  </>
                ) : (
                  <>
                    <AlertCircle className="w-3 h-3 mr-1" />
                    Not Connected
                  </>
                )}
              </Badge>
            </div>
            {lastEvent && (
              <div className="p-3 rounded-lg bg-slate-700/50 border border-slate-600">
                <div className="text-xs text-slate-400 mb-1">Last Event</div>
                <div className="text-sm text-white font-mono">{lastEvent}</div>
              </div>
            )}
          </CardContent>
        </Card>

        {/* Controls */}
        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          {/* Intensity & Duration */}
          <Card className="bg-slate-800/50 border-slate-700">
            <CardHeader>
              <CardTitle className="text-white flex items-center gap-2">
                <Settings className="w-5 h-5 text-purple-400" />
                Haptic Parameters
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-6">
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-slate-400">Intensity</span>
                  <span className="text-white font-mono">{(intensity[0] * 100).toFixed(0)}%</span>
                </div>
                <Slider
                  value={intensity}
                  onValueChange={setIntensity}
                  min={0}
                  max={1}
                  step={0.1}
                  className="w-full"
                />
              </div>
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-slate-400">Duration (ms)</span>
                  <span className="text-white font-mono">{duration[0]}ms</span>
                </div>
                <Slider
                  value={duration}
                  onValueChange={setDuration}
                  min={50}
                  max={1000}
                  step={50}
                  className="w-full"
                />
              </div>
            </CardContent>
          </Card>

          {/* Quick Actions */}
          <Card className="bg-slate-800/50 border-slate-700">
            <CardHeader>
              <CardTitle className="text-white flex items-center gap-2">
                <Zap className="w-5 h-5 text-yellow-400" />
                Quick Actions
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-3">
              <Button
                className="w-full bg-green-500 hover:bg-green-600 text-white"
                onClick={triggerSuccess}
                disabled={!isSupported}
              >
                <CheckCircle className="w-4 h-4 mr-2" />
                Success Pulse
              </Button>
              <div className="grid grid-cols-3 gap-2">
                <Button
                  variant="outline"
                  className="border-slate-600 text-slate-300"
                  onClick={() => triggerCollision('light')}
                  disabled={!isSupported}
                >
                  Light
                </Button>
                <Button
                  variant="outline"
                  className="border-slate-600 text-slate-300"
                  onClick={() => triggerCollision('medium')}
                  disabled={!isSupported}
                >
                  Medium
                </Button>
                <Button
                  variant="outline"
                  className="border-slate-600 text-slate-300"
                  onClick={() => triggerCollision('heavy')}
                  disabled={!isSupported}
                >
                  Heavy
                </Button>
              </div>
              <Button
                variant="outline"
                className="w-full border-slate-600 text-slate-300"
                onClick={triggerGripResistance}
                disabled={!isSupported}
              >
                <Vibrate className="w-4 h-4 mr-2" />
                Grip Resistance
              </Button>
            </CardContent>
          </Card>
        </div>

        {/* Event Types */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white">Haptic Event Types</CardTitle>
            <CardDescription>Test different haptic feedback patterns</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
              <Button
                variant="outline"
                className="h-24 flex-col border-slate-600 text-slate-300"
                onClick={() => triggerHaptic('collision')}
                disabled={!isSupported}
              >
                <Vibrate className="w-6 h-6 mb-2" />
                Collision
              </Button>
              <Button
                variant="outline"
                className="h-24 flex-col border-slate-600 text-slate-300"
                onClick={() => triggerHaptic('resistance')}
                disabled={!isSupported}
              >
                <Volume2 className="w-6 h-6 mb-2" />
                Resistance
              </Button>
              <Button
                variant="outline"
                className="h-24 flex-col border-slate-600 text-slate-300"
                onClick={() => triggerHaptic('pulse')}
                disabled={!isSupported}
              >
                <Zap className="w-6 h-6 mb-2" />
                Pulse
              </Button>
              <Button
                variant="outline"
                className="h-24 flex-col border-slate-600 text-slate-300"
                onClick={() => triggerHaptic('continuous')}
                disabled={!isSupported}
              >
                <Gamepad2 className="w-6 h-6 mb-2" />
                Continuous
              </Button>
            </div>
          </CardContent>
        </Card>

        {/* Instructions */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white">Instructions</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 text-sm text-slate-400">
            <p>1. Connect a gamepad (Xbox, PlayStation, or compatible controller) to your device</p>
            <p>2. The browser will automatically detect the gamepad</p>
            <p>3. Use the controls above to test different haptic feedback patterns</p>
            <p>4. Adjust intensity and duration sliders to customize the feedback</p>
            <p>5. If no gamepad is available, audio-based spectral haptics will be used</p>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}
