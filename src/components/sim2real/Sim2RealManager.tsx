'use client'

import { useState, useEffect } from 'react'
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Toggle } from '@/components/ui/toggle'
import { Slider } from '@/components/ui/slider'
import { motion } from 'framer-motion'
import {
  Cpu, Server, Zap, Play, Pause, RotateCw,
  ArrowRightLeft, ArrowRightRight, RefreshCw,
  CheckCircle, XCircle, Layers, Globe
} from 'lucide-react'
import { createDigitalTwinEngine } from '@/lib/digital-twin/twin-engine'
import { createRobotConnection, RobotConnection } from '@/lib/iot/robot-connection'

type Domain = 'simulation' | 'reality'

export function Sim2RealManager() {
  const [activeDomain, setActiveDomain] = useState<Domain>('simulation')
  const [isTraining, setIsTraining] = useState(false)
  const [trainingProgress, setTrainingProgress] = useState(0)
  const [domainSyncStatus, setDomainSyncStatus] = useState<'synced' | 'syncing' | 'error'>('synced')
  const [isRealityReady, setIsRealityReady] = useState(false)
  const [robotConnection, setRobotConnection] = useState<RobotConnection | null>(null)
  const [telemetry, setTelemetry] = useState<any>(null)
  
  // Initialize Digital Twin
  const twinEngine = createDigitalTwinEngine()
  
  useEffect(() => {
    // Register a sim twin for demonstration
    twinEngine.registerSimulationTwin('sim-bot-1', 'Simulation Bot', {
      position: { x: 0, y: 0, z: 0 }
    })

    // Initialize robot connection
    const initRobotConnection = async () => {
      try {
        // Get configuration from environment variables or use defaults
        const robotId = process.env.NEXT_PUBLIC_ROBOT_ID || 'unitree-g1-1'
        const robotType = (process.env.NEXT_PUBLIC_ROBOT_TYPE || 'unitree-g1') as 'unitree-g1' | 'boston-atlas' | 'tesla-optimus' | 'agility-digit' | 'generic'
        const connectionType = (process.env.NEXT_PUBLIC_ROBOT_CONNECTION_TYPE || 'websocket') as 'mqtt' | 'websocket' | 'rest' | 'ros2'
        const robotHost = process.env.NEXT_PUBLIC_ROBOT_HOST || 'localhost'
        const robotPort = parseInt(process.env.NEXT_PUBLIC_ROBOT_PORT || '8080', 10)

        const connection = createRobotConnection({
          robotId,
          type: robotType,
          connectionType,
          host: robotHost,
          port: robotPort
        })

        // Subscribe to telemetry
        connection.subscribe('telemetry', (data: any) => {
          setTelemetry(data)
          
          // Update Digital Twin with physical robot state
          twinEngine.updateTwinState('physical-bot-1', {
            battery: data.battery,
            status: 'connected'
          })
        })

        // Try to connect
        const connected = await connection.connect()
        if (connected) {
          setRobotConnection(connection)
          setIsRealityReady(true)
          
          // Register physical robot with Digital Twin
          twinEngine.registerPhysicalRobot('physical-bot-1', 'Physical Unitree G1', connection)
        } else {
          setIsRealityReady(false)
        }
      } catch (error) {
        console.error('Failed to connect to robot:', error)
        setIsRealityReady(false)
      }
    }

    initRobotConnection()
    
    // Check reality connection status periodically
    const checkReality = setInterval(() => {
      if (robotConnection) {
        const status = robotConnection.getStatus()
        setIsRealityReady(status.connected)
      }
    }, 2000)

    return () => {
      clearInterval(checkReality)
      if (robotConnection) {
        robotConnection.disconnect()
      }
    }
  }, [])

  const handleSwitchDomain = async (domain: Domain) => {
    if (domain === 'reality' && !isRealityReady) {
      alert('Physical robot not connected. Please check connection.')
      return
    }
    
    setActiveDomain(domain)
    setDomainSyncStatus('syncing')
    
    // Sync state between domains
    if (domain === 'reality' && robotConnection) {
      // Get current simulation state from Digital Twin
      const simTwin = twinEngine.getTwin('sim-bot-1')
      if (simTwin) {
        // Send simulation state to physical robot
        await robotConnection.sendCommand('sync_state', {
          position: simTwin.position,
          rotation: simTwin.rotation
        })
      }
    } else if (domain === 'simulation' && robotConnection && telemetry) {
      // Update simulation with physical robot state
      const physicalTwin = twinEngine.getTwin('physical-bot-1')
      if (physicalTwin) {
        twinEngine.updateTwinState('sim-bot-1', {
          position: physicalTwin.position,
          rotation: physicalTwin.rotation
        })
      }
    }
    
    // Simulate transfer delay
    setTimeout(() => {
      setDomainSyncStatus('synced')
    }, 1000)
  }

  const handleStartTraining = () => {
    setIsTraining(true)
    setTrainingProgress(0)
    
    // Simulate training loop
    const interval = setInterval(() => {
      setTrainingProgress(prev => {
        if (prev >= 100) {
          clearInterval(interval)
          setIsTraining(false)
          return prev
        }
        return prev + 1
      })
    }, 50)
  }

  return (
    <div className="p-6 space-y-6 bg-[#0d0d1a]/50 backdrop-blur-sm min-h-[500px]">
      {/* Header */}
      <div className="flex items-center justify-between mb-6">
        <div>
          <h2 className="text-2xl font-bold text-white">Sim2Real Manager</h2>
          <p className="text-slate-400">Bridge the gap between simulation and physical reality</p>
        </div>
        <Button variant="outline" size="sm" className="border-slate-600 text-slate-300">
          <RefreshCw className="w-4 h-4" />
        </Button>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        {/* Domain Switcher */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white flex items-center gap-2">
              <Layers className="w-5 h-5 text-blue-400" />
              Environment Control
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-6">
            <div className="flex gap-4">
              <Button
                variant={activeDomain === 'simulation' ? 'default' : 'outline'}
                className={`flex-1 h-24 flex-col justify-center gap-2 ${
                  activeDomain === 'simulation' ? 'bg-blue-500' : 'bg-slate-700 hover:bg-slate-600'
                }`}
                onClick={() => handleSwitchDomain('simulation')}
              >
                <Cpu className="w-8 h-8 text-white" />
                <span className="text-lg font-semibold">Simulation</span>
                <Badge className={activeDomain === 'simulation' ? 'bg-white/20 text-white' : 'bg-slate-600 text-slate-300'}>
                  Safe
                </Badge>
              </Button>
              
              <Button
                variant={activeDomain === 'reality' ? 'default' : 'outline'}
                className={`flex-1 h-24 flex-col justify-center gap-2 ${
                  activeDomain === 'reality' ? 'bg-orange-500' : 'bg-slate-700 hover:bg-slate-600'
                }`}
                onClick={() => handleSwitchDomain('reality')}
                disabled={!isRealityReady}
              >
                <Globe className="w-8 h-8 text-white" />
                <span className="text-lg font-semibold">Reality</span>
                <Badge className={isRealityReady ? 'bg-white/20 text-white' : 'bg-red-500/20 text-red-400'}>
                  {isRealityReady ? 'Online' : 'Offline'}
                </Badge>
              </Button>
            </div>

            {/* Sync Status */}
            <div className="p-4 rounded-lg bg-slate-700/50 border border-slate-600">
              <div className="flex items-center justify-between">
                <div>
                  <div className="text-sm text-slate-400 mb-1">Sync Status</div>
                  <div className="flex items-center gap-2">
                    {domainSyncStatus === 'synced' && <CheckCircle className="w-5 h-5 text-green-400" />}
                    {domainSyncStatus === 'syncing' && <RefreshCw className="w-5 h-5 text-blue-400 animate-spin" />}
                    {domainSyncStatus === 'error' && <XCircle className="w-5 h-5 text-red-400" />}
                    <span className="text-sm font-medium text-white capitalize">{domainSyncStatus}</span>
                  </div>
                </div>
                <Badge variant="outline" className="border-slate-500 text-slate-400">
                  <Zap className="w-3 h-3" />
                  20Hz
                </Badge>
              </div>
            </div>
          </CardContent>
        </Card>

        {/* Transfer Training Pipeline */}
        <Card className="bg-slate-800/50 border-slate-700">
          <CardHeader>
            <CardTitle className="text-white flex items-center gap-2">
              <RotateCw className="w-5 h-5 text-purple-400" />
              Sim2Real Transfer
            </CardTitle>
            <CardDescription>
              Train policies in simulation, deploy to reality.
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-6">
            <div className="p-4 rounded-lg bg-slate-700/50 border border-slate-600 space-y-3">
              <div className="flex justify-between items-center">
                <span className="text-sm text-slate-400">Domain Randomization</span>
                <Toggle defaultChecked={true} />
              </div>
              <div className="flex justify-between items-center">
                <span className="text-sm text-slate-400">System ID</span>
                <span className="text-sm font-mono text-white">Unitree-G1-v2</span>
              </div>
            </div>

            <div className="space-y-2">
              <div className="flex justify-between text-sm">
                <span className="text-slate-400">Training Progress</span>
                <span className="text-white font-mono">{trainingProgress}%</span>
              </div>
              <div className="w-full bg-slate-600 rounded-full h-2 overflow-hidden">
                <motion.div
                  className="h-full bg-gradient-to-r from-blue-500 to-purple-500"
                  initial={{ width: '0%' }}
                  animate={{ width: `${trainingProgress}%` }}
                  transition={{ duration: 0.3 }}
                />
              </div>
            </div>

            <div className="grid grid-cols-2 gap-3">
              <Button 
                className="w-full bg-green-500 hover:bg-green-600 text-white"
                onClick={handleStartTraining}
                disabled={isTraining}
              >
                <Play className="w-4 h-4 mr-2" />
                {isTraining ? 'Training...' : 'Start Training'}
              </Button>
              <Button 
                variant="outline" 
                className="w-full border-slate-600 text-slate-300"
                onClick={() => setIsTraining(false)}
              >
                <Pause className="w-4 h-4 mr-2" />
                Stop
              </Button>
            </div>

            <div className="grid grid-cols-2 gap-3 pt-4 border-t border-slate-600">
              <div className="text-center">
                <div className="text-2xl font-bold text-green-400">245</div>
                <div className="text-xs text-slate-400">Sim Episodes</div>
              </div>
              <div className="text-center">
                <div className="text-2xl font-bold text-blue-400">98%</div>
                <div className="text-xs text-slate-400">Success Rate</div>
              </div>
            </div>

            {/* Robot Telemetry */}
            {telemetry && (
              <div className="pt-4 border-t border-slate-600 space-y-2">
                <div className="text-sm font-semibold text-slate-300 mb-2">Physical Robot Status</div>
                <div className="grid grid-cols-2 gap-2 text-xs">
                  <div>
                    <span className="text-slate-400">Battery: </span>
                    <span className="text-white font-mono">{telemetry.battery?.toFixed(1) || 'N/A'}%</span>
                  </div>
                  <div>
                    <span className="text-slate-400">CPU: </span>
                    <span className="text-white font-mono">{telemetry.cpu?.toFixed(1) || 'N/A'}%</span>
                  </div>
                  <div>
                    <span className="text-slate-400">Temp: </span>
                    <span className="text-white font-mono">{telemetry.temperature?.toFixed(1) || 'N/A'}°C</span>
                  </div>
                  <div>
                    <span className="text-slate-400">Latency: </span>
                    <span className="text-white font-mono">{telemetry.latency?.toFixed(0) || 'N/A'}ms</span>
                  </div>
                </div>
              </div>
            )}
          </CardContent>
        </Card>
      </div>

      {/* Data Visualization */}
      <Card className="bg-slate-800/50 border-slate-700">
        <CardHeader>
          <CardTitle className="text-white">Twin Telemetry</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            {[
              { label: 'Sync Latency', value: '15ms', unit: '', color: 'text-green-400' },
              { label: 'Packet Loss', value: '0.01%', unit: '', color: 'text-blue-400' },
              { label: 'Jitter', value: '2ms', unit: '', color: 'text-purple-400' },
              { label: 'State Errors', value: '0', unit: '/h', color: 'text-yellow-400' },
            ].map((stat, idx) => (
              <motion.div
                key={idx}
                initial={{ opacity: 0, scale: 0.9 }}
                animate={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.3, delay: idx * 0.05 }}
                className="p-3 rounded-lg bg-slate-700/50 border border-slate-600"
              >
                <div className="text-xs text-slate-400 mb-1">{stat.label}</div>
                <div className={`text-xl font-bold ${stat.color}`}>
                  {stat.value} <span className="text-xs text-slate-500">{stat.unit}</span>
                </div>
              </motion.div>
            ))}
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
