'use client'

import { useState, useEffect } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Slider } from '@/components/ui/slider'
import Link from 'next/link'
import PhotoRealisticSimulation from '@/components/simulation/PhotoRealisticSimulation'
import AdvancedSimulation from '@/components/simulation/AdvancedSimulation'
import NeoVerseSimulation from '@/components/simulation/NeoVerseSimulation'
import { PerformanceMonitor } from '@/components/simulation/PerformanceMonitor'

export default function Simulation4DPage() {
  const [isRunning, setIsRunning] = useState(true)
  const [simulationSpeed, setSimulationSpeed] = useState(1.0)
  const [selectedRobot, setSelectedRobot] = useState('unitree-g1')
  const [cameraMode, setCameraMode] = useState('auto')
  const [useAdvancedMode, setUseAdvancedMode] = useState(true)
  const [useNeoVerse, setUseNeoVerse] = useState(true)
  const [isMounted, setIsMounted] = useState(false)
  const [showSensors, setShowSensors] = useState(true)
  const [showPathPlanning, setShowPathPlanning] = useState(true)
  
  // Initialize with fixed values to prevent hydration mismatch
  const initialJoints = Array(8).fill(0).map((_, i) => ({ 
    name: 'Joint ' + (i + 1), 
    position: 45 + (i * 5) // Fixed initial values instead of random
  }))
  
  const [telemetry, setTelemetry] = useState({
    joints: initialJoints,
    battery: 98,
    cpu: 42,
    latency: 15,
    fps: 60
  })

  // Extract joint angles for 3D simulation
  const jointAngles = telemetry.joints.map(j => j.position)

  // Initialize random values only on client after mount
  useEffect(() => {
    setIsMounted(true)
    // Initialize with random values after hydration
    setTelemetry(t => ({
      ...t,
      joints: Array(8).fill(0).map((_, i) => ({ 
        name: 'Joint ' + (i + 1), 
        position: Math.random() * 90 
      }))
    }))
  }, [])
  
  const robots = [
    { id: 'unitree-g1', name: 'Unitree G1', height: '1.8m', weight: '45kg', capability: 'Dynamic Balance', color: '#00d4ff' },
    { id: 'boston-atlas', name: 'Boston Dynamics Atlas', height: '2.0m', weight: '120kg', capability: 'Advanced AI', color: '#ff6b6b' },
    { id: 'tesla-optimus', name: 'Tesla Optimus Gen-3', height: '2.5m', weight: '200kg', capability: 'Full Autonomy', color: '#a3ffb3' },
    { id: 'agility-digit', name: 'Agility Robotics Digit', height: '1.6m', weight: '43kg', capability: 'Parkour', color: '#ffa500' },
  ]

  useEffect(() => {
    if (isRunning) {
      const interval = setInterval(() => {
        setTelemetry(t => ({
          ...t,
          joints: t.joints.map(j => ({ ...j, position: (j.position + Math.random() * 5 - 2.5) % 180 })),
          battery: Math.max(0, t.battery - 0.001 * simulationSpeed),
          cpu: 40 + Math.random() * 10,
          fps: 55 + Math.random() * 10
        }))
      }, 100)
      return () => clearInterval(interval)
    }
  }, [isRunning, simulationSpeed])

  return (
    <div className="min-h-screen relative overflow-hidden bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900">
      {/* State-of-the-art 3D Simulation */}
      <div className="absolute inset-0 z-0">
        {useNeoVerse ? (
          <NeoVerseSimulation
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={simulationSpeed}
            jointAngles={jointAngles}
            config={{
              enabled: true,
              temporalConsistency: true,
              spatialModeling: true,
              realWorldPhysics: true,
              cameraTracking: true
            }}
          />
        ) : useAdvancedMode ? (
          <AdvancedSimulation
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={simulationSpeed}
            cameraMode={cameraMode}
            jointAngles={jointAngles}
            showSensors={showSensors}
            showPathPlanning={showPathPlanning}
          />
        ) : (
          <PhotoRealisticSimulation
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={simulationSpeed}
            cameraMode={cameraMode}
            jointAngles={jointAngles}
          />
        )}
      </div>

      <nav className="fixed top-0 left-0 right-0 z-50 border-b border-white/10 bg-black/30 backdrop-blur-xl">
        <div className="container mx-auto px-4 py-3">
          <div className="flex items-center justify-between">
            <Link href="/student" className="flex items-center gap-2">
              <span className="text-2xl">🤖</span>
              <div>
                <div className="text-lg font-bold bg-gradient-to-r from-cyan-400 to-blue-500 bg-clip-text text-transparent">4D SPATIAL</div>
                <div className="text-xs text-cyan-300">Real 3D Interactive Simulation</div>
              </div>
            </Link>
            <div className="flex items-center gap-4">
              <Link href="/catalog" className="text-sm text-white/80 hover:text-white">Courses</Link>
              <Link href="/learning-paths" className="text-sm text-white/80 hover:text-white">Learning Paths</Link>
              <Link href="/ai-video/generate" className="text-sm text-white/80 hover:text-white">AI Video Studio</Link>
            </div>
          </div>
        </div>
      </nav>

      <div className="absolute left-4 top-20 bottom-4 w-80 flex flex-col gap-3 overflow-y-auto z-40">
        <Card className="bg-black/40 backdrop-blur-2xl border-white/20">
          <CardHeader className="bg-gradient-to-r from-blue-500/30 to-cyan-500/30 border-b border-white/20">
            <CardTitle className="text-white text-lg">Robot Selection</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 p-2">
            {robots.map((robot) => (
              <button key={robot.id} onClick={() => setSelectedRobot(robot.id)} className={'w-full text-left p-3 rounded-xl border-2 ' + (selectedRobot === robot.id ? 'border-cyan-400 bg-cyan-500/30' : 'border-white/20 bg-black/10')}>
                <div className="font-semibold text-white">{robot.name}</div>
                <div className="text-xs text-white/50">{robot.height} • {robot.weight}</div>
                <Badge variant="outline" className="text-xs mt-1 border-white/30">{robot.capability}</Badge>
              </button>
            ))}
          </CardContent>
        </Card>

        <Card className="bg-black/40 backdrop-blur-2xl border-white/20">
          <CardHeader className="bg-gradient-to-r from-purple-500/30 to-pink-500/30 border-b border-white/20">
            <CardTitle className="text-white text-lg">3D Controls</CardTitle>
          </CardHeader>
          <CardContent className="space-y-3 p-2">
            <div className="flex items-center justify-between">
              <span className="text-white text-xs">NeoVerse 4D</span>
              <Button 
                size="sm" 
                onClick={() => setUseNeoVerse(!useNeoVerse)} 
                className={useNeoVerse ? "bg-green-500" : "bg-gray-500"}
              >
                {useNeoVerse ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-white text-xs">Advanced Mode</span>
              <Button 
                size="sm" 
                onClick={() => setUseAdvancedMode(!useAdvancedMode)} 
                className={useAdvancedMode && !useNeoVerse ? "bg-green-500" : "bg-gray-500"}
                disabled={useNeoVerse}
              >
                {useAdvancedMode ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-white text-xs">Sensors</span>
              <Button 
                size="sm" 
                onClick={() => setShowSensors(!showSensors)} 
                className={showSensors ? "bg-green-500" : "bg-gray-500"}
              >
                {showSensors ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-white text-xs">Path Planning</span>
              <Button 
                size="sm" 
                onClick={() => setShowPathPlanning(!showPathPlanning)} 
                className={showPathPlanning ? "bg-green-500" : "bg-gray-500"}
              >
                {showPathPlanning ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-white">Auto Rotate: {cameraMode === 'auto' ? 'On' : 'Off'}</span>
              <Button size="sm" onClick={() => setCameraMode(cameraMode === 'auto' ? 'manual' : 'auto')} className="bg-blue-500">Toggle</Button>
            </div>
            <div>
              <div className="flex justify-between text-white text-sm mb-2">
                <span>Speed</span>
                <span>{simulationSpeed.toFixed(1)}x</span>
              </div>
              <Slider value={[simulationSpeed]} onValueChange={([v]) => setSimulationSpeed(v)} min={0.1} max={3} step={0.1} />
            </div>
            <div className="flex gap-2">
              <Button size="sm" onClick={() => setIsRunning(!isRunning)} className="flex-1">{isRunning ? 'Pause' : 'Play'}</Button>
              <Button size="sm" onClick={() => setSimulationSpeed(1.0)}>Reset</Button>
            </div>
          </CardContent>
        </Card>
      </div>

      <div className="absolute right-4 top-20 w-80 flex flex-col gap-3 overflow-y-auto z-40">
        <Card className="bg-black/40 backdrop-blur-2xl border-white/20">
          <CardHeader className="bg-gradient-to-r from-red-500/30 to-orange-500/30 border-b border-white/20">
            <CardTitle className="text-white text-lg">Real-time Telemetry</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 p-2">
            <div className="grid grid-cols-2 gap-2">
              <div className="bg-white/5 rounded p-2"><div className="text-white/50 text-xs">Battery</div><div className="text-green-400 font-mono">{telemetry.battery.toFixed(1)}%</div></div>
              <div className="bg-white/5 rounded p-2"><div className="text-white/50 text-xs">CPU</div><div className="text-blue-400 font-mono">{telemetry.cpu.toFixed(0)}%</div></div>
              <div className="bg-white/5 rounded p-2"><div className="text-white/50 text-xs">Latency</div><div className="text-blue-400 font-mono">{telemetry.latency}ms</div></div>
              <div className="bg-white/5 rounded p-2"><div className="text-white/50 text-xs">FPS</div><div className="text-green-400 font-mono">{telemetry.fps.toFixed(0)}</div></div>
            </div>
            <div>
              <div className="text-white text-sm mb-2">8 Joint Positions</div>
              {telemetry.joints.map((joint, i) => (
                <div key={i} className="flex items-center gap-2">
                  <div className="w-16 text-xs text-white/70">{joint.name}</div>
                  <div className="flex-1 bg-white/5 rounded h-2">
                    <div className="bg-gradient-to-r from-cyan-500 to-blue-500 h-full" style={{ width: Math.abs(joint.position) + '%' }}></div>
                  </div>
                  <div className="w-10 text-xs text-cyan-400 font-mono">{joint.position.toFixed(0)}°</div>
                </div>
              ))}
            </div>
          </CardContent>
        </Card>

        <Card className="bg-black/40 backdrop-blur-2xl border-white/20">
          <CardContent className="p-3">
            <Link href="/ai-video/generate">
              <Button className="w-full bg-gradient-to-r from-cyan-500 to-blue-600 text-white">
                Generate AI Video
                <div className="text-xs text-white/70">7 AI Video Models</div>
              </Button>
            </Link>
          </CardContent>
        </Card>
      </div>

      <div className="fixed top-20 right-4 z-30">
        <Link href="/student">
          <Button variant="ghost" size="sm" className="bg-black/10 border-white/20 text-white">Dashboard</Button>
        </Link>
      </div>

      {/* Performance Monitor */}
      <PerformanceMonitor />
    </div>
  )
}
