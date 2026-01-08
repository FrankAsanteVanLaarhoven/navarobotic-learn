'use client'

import { useState, useEffect } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Slider } from '@/components/ui/slider'
import Link from 'next/link'
import { Cpu } from 'lucide-react'
import PhotoRealisticSimulation from '@/components/simulation/PhotoRealisticSimulation'
import AdvancedSimulation from '@/components/simulation/AdvancedSimulation'
import NeoVerseSimulation from '@/components/simulation/NeoVerseSimulation'
import { PerformanceMonitor } from '@/components/simulation/PerformanceMonitor'
import { AnimatedLogo } from '@/components/AnimatedLogo'

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
    { id: 'unitree-g1', name: 'Unitree G1', height: '1.8m', weight: '45kg', capability: 'Dynamic Balance', color: '#2E7DFF' },
    { id: 'boston-atlas', name: 'Boston Dynamics Atlas', height: '2.0m', weight: '120kg', capability: 'Advanced AI', color: '#00C2A8' },
    { id: 'tesla-optimus', name: 'Tesla Optimus Gen-3', height: '2.5m', weight: '200kg', capability: 'Full Autonomy', color: '#FFD166' },
    { id: 'agility-digit', name: 'Agility Robotics Digit', height: '1.6m', weight: '43kg', capability: 'Parkour', color: '#2E7DFF' },
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
    <div className="min-h-screen relative overflow-hidden bg-background">
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

      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 py-3">
          <div className="flex items-center justify-between">
            <Link href="/" className="flex items-center gap-2">
              <AnimatedLogo size="sm" />
              <div>
                <div className="text-lg font-bold">Rovyn Simulation</div>
                <div className="text-xs text-muted-foreground">Real-time 3D Robot Simulation</div>
              </div>
            </Link>
            <div className="flex items-center gap-4">
              <Link href="/catalog" className="text-sm text-muted-foreground hover:text-foreground transition-colors">Courses</Link>
              <Link href="/learning-paths" className="text-sm text-muted-foreground hover:text-foreground transition-colors">Learning Paths</Link>
              <Link href="/ai-video/generate" className="text-sm text-primary hover:text-primary transition-colors font-semibold">AI Video Studio</Link>
            </div>
          </div>
        </div>
      </nav>

      <div className="absolute left-4 top-20 bottom-4 w-80 flex flex-col gap-3 overflow-y-auto z-40">
        <Card className="glass border-border/50">
          <CardHeader className="border-b border-border/50">
            <CardTitle className="text-lg">Robot Selection</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 p-4">
            {robots.map((robot) => (
              <button 
                key={robot.id} 
                onClick={() => setSelectedRobot(robot.id)} 
                className={`w-full text-left p-3 rounded-xl border-2 transition-all ${
                  selectedRobot === robot.id 
                    ? 'border-primary bg-primary/10' 
                    : 'border-border/50 bg-card hover:border-primary/50'
                }`}
              >
                <div className="font-semibold">{robot.name}</div>
                <div className="text-xs text-muted-foreground">{robot.height} • {robot.weight}</div>
                <Badge variant="outline" className="text-xs mt-1">{robot.capability}</Badge>
              </button>
            ))}
          </CardContent>
        </Card>

        <Card className="glass border-border/50">
          <CardHeader className="border-b border-border/50">
            <CardTitle className="text-lg">3D Controls</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4 p-4">
            <div className="flex items-center justify-between">
              <span className="text-sm">NeoVerse 4D</span>
              <Button 
                size="sm" 
                onClick={() => setUseNeoVerse(!useNeoVerse)} 
                variant={useNeoVerse ? "default" : "outline"}
                className={useNeoVerse ? "" : ""}
              >
                {useNeoVerse ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm">Advanced Mode</span>
              <Button 
                size="sm" 
                onClick={() => setUseAdvancedMode(!useAdvancedMode)} 
                variant={useAdvancedMode && !useNeoVerse ? "default" : "outline"}
                disabled={useNeoVerse}
              >
                {useAdvancedMode ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm">Sensors</span>
              <Button 
                size="sm" 
                onClick={() => setShowSensors(!showSensors)} 
                variant={showSensors ? "default" : "outline"}
              >
                {showSensors ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm">Path Planning</span>
              <Button 
                size="sm" 
                onClick={() => setShowPathPlanning(!showPathPlanning)} 
                variant={showPathPlanning ? "default" : "outline"}
              >
                {showPathPlanning ? 'ON' : 'OFF'}
              </Button>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm">Auto Rotate: {cameraMode === 'auto' ? 'On' : 'Off'}</span>
              <Button 
                size="sm" 
                onClick={() => setCameraMode(cameraMode === 'auto' ? 'manual' : 'auto')} 
                variant="outline"
              >
                Toggle
              </Button>
            </div>
            <div>
              <div className="flex justify-between text-sm mb-2">
                <span>Speed</span>
                <span className="font-mono">{simulationSpeed.toFixed(1)}x</span>
              </div>
              <Slider value={[simulationSpeed]} onValueChange={([v]) => setSimulationSpeed(v)} min={0.1} max={3} step={0.1} />
            </div>
            <div className="flex gap-2">
              <Button size="sm" onClick={() => setIsRunning(!isRunning)} className="flex-1" variant={isRunning ? "default" : "outline"}>
                {isRunning ? 'Pause' : 'Play'}
              </Button>
              <Button size="sm" onClick={() => setSimulationSpeed(1.0)} variant="outline">Reset</Button>
            </div>
          </CardContent>
        </Card>
      </div>

      <div className="absolute right-4 top-20 w-80 flex flex-col gap-3 overflow-y-auto z-40">
        <Card className="glass border-border/50">
          <CardHeader className="border-b border-border/50">
            <CardTitle className="text-lg">Real-time Telemetry</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4 p-4">
            <div className="grid grid-cols-2 gap-3">
              <div className="bg-muted/50 rounded-lg p-3 border border-border/50">
                <div className="text-xs text-muted-foreground mb-1">Battery</div>
                <div className="text-lg font-mono font-semibold text-primary">{telemetry.battery.toFixed(1)}%</div>
              </div>
              <div className="bg-muted/50 rounded-lg p-3 border border-border/50">
                <div className="text-xs text-muted-foreground mb-1">CPU</div>
                <div className="text-lg font-mono font-semibold text-secondary">{telemetry.cpu.toFixed(0)}%</div>
              </div>
              <div className="bg-muted/50 rounded-lg p-3 border border-border/50">
                <div className="text-xs text-muted-foreground mb-1">Latency</div>
                <div className="text-lg font-mono font-semibold">{telemetry.latency}ms</div>
              </div>
              <div className="bg-muted/50 rounded-lg p-3 border border-border/50">
                <div className="text-xs text-muted-foreground mb-1">FPS</div>
                <div className="text-lg font-mono font-semibold text-primary">{telemetry.fps.toFixed(0)}</div>
              </div>
            </div>
            <div>
              <div className="text-sm font-medium mb-3">Joint Positions</div>
              <div className="space-y-2">
                {telemetry.joints.map((joint, i) => (
                  <div key={i} className="flex items-center gap-3">
                    <div className="w-16 text-xs text-muted-foreground font-mono">{joint.name}</div>
                    <div className="flex-1 bg-muted/50 rounded-full h-2 border border-border/50 overflow-hidden">
                      <div 
                        className="bg-primary h-full rounded-full transition-all" 
                        style={{ width: `${Math.min(100, Math.abs(joint.position) / 1.8)}%` }}
                      />
                    </div>
                    <div className="w-12 text-xs font-mono text-right">{joint.position.toFixed(0)}°</div>
                  </div>
                ))}
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="glass border-border/50">
          <CardContent className="p-4">
            <Link href="/ai-video/generate">
              <Button className="w-full">
                Generate AI Video
                <div className="text-xs text-muted-foreground mt-1">7 AI Video Models</div>
              </Button>
            </Link>
          </CardContent>
        </Card>
      </div>

      <div className="fixed top-20 right-[22rem] z-30">
        <Link href="/student">
          <Button variant="outline" size="sm" className="glass">Dashboard</Button>
        </Link>
      </div>

      {/* Performance Monitor */}
      <PerformanceMonitor />
    </div>
  )
}
