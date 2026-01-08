/**
 * Multi-Robot Scene Manager
 * Manages multiple robots in a single 3D scene with physics
 */

'use client'

import { useRef, useState, useEffect, useCallback } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Environment, ContactShadows, Stage, Sky, Html } from '@react-three/drei'
import * as THREE from 'three'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Slider } from '@/components/ui/slider'
import {
  Maximize, Minimize, RotateCw, Settings,
  Users, Grid3x3, Map, RefreshCw
} from 'lucide-react'
import { PhysicsEngine } from '@/lib/physics/physics-engine'
import { createDigitalTwinEngine, TwinState } from '@/lib/digital-twin/twin-engine'

interface RobotSceneConfig {
  id: string
  name: string
  type: 'humanoid' | 'bipedal' | 'mobile'
  position: { x: number; y: number; z: number }
  rotation: { x: number; y: number; z: number }
  scale: number
  color: string
  url?: string
}

interface MultiRobotSceneProps {
  robots: RobotSceneConfig[]
  onRobotSelect?: (robotId: string) => void
  onJointUpdate?: (robotId: string, joint: string, angles: { x: number; y: number; z: number }) => void
  enablePhysics?: boolean
  showGrid?: boolean
  showSky?: boolean
  lightMode?: 'day' | 'night'
}

function SceneContent({
  robots,
  selectedRobot,
  onRobotSelect,
  onJointUpdate,
  enablePhysics,
  showGrid,
  showSky,
  lightMode,
  physicsEngine
}: {
  robots: RobotSceneConfig[]
  selectedRobot: string | null
  onRobotSelect?: (robotId: string) => void
  onJointUpdate?: (robotId: string, joint: string, angles: { x: number; y: number; z: number }) => void
  enablePhysics?: boolean
  showGrid?: boolean
  showSky?: boolean
  lightMode?: 'day' | 'night'
  physicsEngine: PhysicsEngine | null
}) {
  const { scene, camera } = useThree()
  const robotRefs = useRef<Map<string, THREE.Group>>(new Map())

  // Update physics
  useFrame(() => {
    if (enablePhysics && physicsEngine) {
      physicsEngine.step()
    }
  })

  const createRobotMesh = useCallback((robot: RobotSceneConfig) => {
    return (
      <group
        key={robot.id}
        ref={(ref) => {
          if (ref) robotRefs.current.set(robot.id, ref as THREE.Group)
        }}
        position={[
          twinStates.get(robot.id)?.position.x ?? robot.position.x,
          twinStates.get(robot.id)?.position.y ?? robot.position.y,
          twinStates.get(robot.id)?.position.z ?? robot.position.z
        ]}
        rotation={[robot.rotation.x, robot.rotation.y, robot.rotation.z]}
        scale={robot.scale}
        onClick={() => onRobotSelect?.(robot.id)}
        onPointerOver={() => document.body.style.cursor = 'pointer'}
        onPointerOut={() => document.body.style.cursor = 'default'}
      >
        {/* Robot Base */}
        <mesh castShadow>
          <boxGeometry args={[0.8 * robot.scale, 0.4 * robot.scale, 0.4 * robot.scale]} />
          <meshStandardMaterial 
            color={robot.color} 
            metalness={0.8}
            roughness={0.2}
          />
        </mesh>

        {/* Robot Body */}
        <mesh position={[0, 0.4 * robot.scale, 0]} castShadow>
          <boxGeometry args={[0.6 * robot.scale, 0.8 * robot.scale, 0.4 * robot.scale]} />
          <meshStandardMaterial 
            color={new THREE.Color(robot.color).multiplyScalar(0.9).getHex()} 
            metalness={0.7}
            roughness={0.3}
          />
        </mesh>

        {/* Robot Head */}
        <mesh position={[0, 1.0 * robot.scale, 0]} castShadow>
          <sphereGeometry args={[0.25 * robot.scale, 32, 32]} />
          <meshStandardMaterial 
            color={new THREE.Color(robot.color).multiplyScalar(1.1).getHex()} 
            metalness={0.6}
            roughness={0.4}
          />
        </mesh>

        {/* Arms */}
        <group position={[0, 0.8 * robot.scale, 0]}>
          <mesh position={[-0.3 * robot.scale, 0, 0]} castShadow>
            <cylinderGeometry args={[0.08 * robot.scale, 0.3 * robot.scale, 0.3 * robot.scale, 32]} />
            <meshStandardMaterial 
              color={new THREE.Color(robot.color).multiplyScalar(0.9).getHex()} 
              metalness={0.7}
              roughness={0.3}
            />
          </mesh>
          <mesh position={[0.3 * robot.scale, 0, 0]} castShadow>
            <cylinderGeometry args={[0.08 * robot.scale, 0.3 * robot.scale, 0.3 * robot.scale, 32]} />
            <meshStandardMaterial 
              color={new THREE.Color(robot.color).multiplyScalar(0.9).getHex()} 
              metalness={0.7}
              roughness={0.3}
            />
          </mesh>
        </group>

        {/* Legs */}
        <group position={[0, -0.2 * robot.scale, 0]}>
          <mesh position={[-0.2 * robot.scale, 0, 0]} castShadow>
            <boxGeometry args={[0.12 * robot.scale, 0.4 * robot.scale, 0.12 * robot.scale]} />
            <meshStandardMaterial 
              color={new THREE.Color(robot.color).multiplyScalar(0.9).getHex()} 
              metalness={0.7}
              roughness={0.3}
            />
          </mesh>
          <mesh position={[0.2 * robot.scale, 0, 0]} castShadow>
            <boxGeometry args={[0.12 * robot.scale, 0.4 * robot.scale, 0.12 * robot.scale]} />
            <meshStandardMaterial 
              color={new THREE.Color(robot.color).multiplyScalar(0.9).getHex()} 
              metalness={0.7}
              roughness={0.3}
            />
          </mesh>
        </group>

        {/* Selection Indicator */}
        {selectedRobot === robot.id && (
          <Html position={[0, 2.5 * robot.scale, 0]} center>
            <div className="p-2 bg-primary/20 backdrop-blur-sm border border-primary/50 rounded-lg">
              <div className="text-xs text-primary font-medium">
                {robot.name}
              </div>
            </div>
          </Html>
        )}
      </group>
    )
  }, [selectedRobot, onRobotSelect])

  return (
    <>
      {/* Lighting */}
      <ambientLight intensity={lightMode === 'day' ? 0.5 : 0.2} />
      <directionalLight
        position={[10, 10, 5]}
        intensity={lightMode === 'day' ? 1 : 0.3}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-near={0.5}
        shadow-camera-far={50}
      />
      {lightMode === 'night' && (
        <>
          <pointLight position={[5, 5, 5]} intensity={0.5} color="#2E7DFF" />
          <pointLight position={[-5, 5, 5]} intensity={0.5} color="#00C2A8" />
        </>
      )}

      {/* Environment */}
      {showSky && <Sky distance={450000} sunPosition={[0, 100, -200]} inclination={0} azimuth={0} />}
      {showGrid && <gridHelper args={[50, 50]} />}
      <Environment preset={lightMode === 'day' ? 'sunset' : 'night'} />
      <ContactShadows opacity={0.5} scale={1} blur={2} far={20} resolution={256} position={[0, -0.5, 0]} />

      {/* Ground Plane */}
      <mesh receiveShadow rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[100, 100]} />
        <meshStandardMaterial 
          color={lightMode === 'day' ? 0x1e293b : 0x0a0a1a} 
          metalness={0.5}
          roughness={0.8}
        />
      </mesh>

      {/* Robots */}
      <Stage center position={[0, 0, 0]} shadows>
        {robots.map(robot => createRobotMesh(robot))}
      </Stage>
    </>
  )
}

export function MultiRobotScene({
  robots,
  onRobotSelect,
  onJointUpdate,
  enablePhysics = false,
  showGrid = true,
  showSky = true,
  lightMode = 'day'
}: MultiRobotSceneProps) {
  const [selectedRobot, setSelectedRobot] = useState<string | null>(null)
  const [autoRotate, setAutoRotate] = useState(false)
  const [showShadows, setShowShadows] = useState(true)
  const [telemetryData, setTelemetryData] = useState<Record<string, any>>({})
  const physicsEngineRef = useRef<PhysicsEngine | null>(null)
  const twinEngineRef = useRef<ReturnType<typeof createDigitalTwinEngine> | null>(null)
  const [twinStates, setTwinStates] = useState<Map<string, TwinState>>(new Map())

  // Initialize Digital Twin Engine and Physics Engine
  useEffect(() => {
    // Initialize Digital Twin Engine
    const twinEngine = createDigitalTwinEngine()
    twinEngineRef.current = twinEngine

    // Register simulation twins for all robots
    robots.forEach(robot => {
      twinEngine.registerSimulationTwin(robot.id, robot.name, {
        position: robot.position
      })

      // Subscribe to twin updates
      twinEngine.subscribe(robot.id, (twin: TwinState) => {
        setTwinStates(prev => {
          const newMap = new Map(prev)
          newMap.set(robot.id, twin)
          return newMap
        })
      })
    })

    // Initialize physics engine
    if (enablePhysics) {
      const engine = new PhysicsEngine({
        gravity: -9.82,
        friction: 0.8,
        restitution: 0.3,
        solverIterations: 10
      })

      // Create ground
      const scene = new THREE.Scene()
      engine.setScene(scene)
      engine.createGround()

      // Create physics bodies for all robots
      robots.forEach(robot => {
        engine.createRobot({
          id: robot.id,
          type: robot.type,
          position: robot.position,
          scale: robot.scale
        })
      })

      engine.start()
      physicsEngineRef.current = engine

      // Update telemetry and sync with Digital Twin
      const interval = setInterval(() => {
        const newTelemetryData: Record<string, any> = {}
        robots.forEach(robot => {
          const position = engine.getRobotPosition(robot.id)
          const velocity = engine.getRobotVelocity(robot.id)
          const angularVelocity = engine.getRobotAngularVelocity(robot.id)

          newTelemetryData[robot.id] = {
            position,
            velocity,
            angularVelocity
          }

          // Update Digital Twin with physics state
          if (twinEngineRef.current) {
            twinEngineRef.current.updateTwinState(robot.id, {
              position: { x: position.x, y: position.y, z: position.z },
              velocity: { x: velocity.x, y: velocity.y, z: velocity.z }
            })
          }
        })
        setTelemetryData(newTelemetryData)
      }, 100)

      return () => {
        clearInterval(interval)
        engine.stop()
        if (twinEngineRef.current) {
          twinEngineRef.current.stopAutoSync()
        }
      }
    }

    return () => {
      if (twinEngineRef.current) {
        twinEngineRef.current.stopAutoSync()
      }
    }
  }, [enablePhysics, robots])

  const handleRobotSelect = useCallback((robotId: string) => {
    setSelectedRobot(robotId)
    onRobotSelect?.(robotId)
  }, [onRobotSelect])

  return (
    <div className="w-full h-full relative bg-background">
      <Canvas
        shadows={showShadows}
        camera={{ position: [5, 5, 10], fov: 45 }}
        gl={{ 
          antialias: true,
          alpha: true,
          preserveDrawingBuffer: true,
          logarithmicDepthBuffer: true,
          powerPreference: 'high-performance'
        }}
        onCreated={({ camera, scene }) => {
          camera.position.set(5, 5, 10)
          scene.background = new THREE.Color(lightMode === 'day' ? 0x87CEEB : 0x000020)
        }}
      >
        <PerspectiveCamera makeDefault position={[5, 5, 10]} fov={45} />
        <OrbitControls
          target={[0, 0, 0]}
          enableDamping
          dampingFactor={0.05}
          autoRotate={autoRotate}
          autoRotateSpeed={2.0}
          maxPolarAngle={Math.PI / 2}
          minPolarAngle={Math.PI / 6}
          maxDistance={50}
          minDistance={2}
        />

        <SceneContent
          robots={robots}
          selectedRobot={selectedRobot}
          onRobotSelect={handleRobotSelect}
          onJointUpdate={onJointUpdate}
          enablePhysics={enablePhysics}
          showGrid={showGrid}
          showSky={showSky}
          lightMode={lightMode}
          physicsEngine={physicsEngineRef.current}
        />

        {/* Control Panel */}
        <Html position={[0, 5, 0]} center distance={5} transform={[-window.innerWidth / 2 + 100, -window.innerHeight / 2 + 100, 0]}>
          <Card className="glass border-border/50 w-96">
            <CardContent className="p-4 space-y-4">
              <div className="flex items-center justify-between mb-4">
                <h3 className="font-semibold">Multi-Robot Scene</h3>
                <Badge className="bg-primary/20 text-primary border-primary/50">
                  <Users className="w-3 h-3 mr-1" />
                  {robots.length} Robots
                </Badge>
              </div>

              {/* Robot Selection */}
              <div className="mb-4">
                <div className="text-sm font-semibold mb-2">Select Robot</div>
                <div className="grid grid-cols-2 gap-2">
                  {robots.map(robot => (
                    <Button
                      key={robot.id}
                      variant={selectedRobot === robot.id ? 'default' : 'outline'}
                      size="sm"
                      onClick={() => handleRobotSelect(robot.id)}
                      className="w-full justify-start"
                    >
                      <div className="w-2 h-2 rounded-full mr-2" style={{ backgroundColor: robot.color }} />
                      {robot.name}
                    </Button>
                  ))}
                </div>
              </div>

              {/* Scene Controls */}
              <div className="space-y-3">
                <div className="text-sm font-semibold mb-2">Scene Controls</div>
                <div className="grid grid-cols-2 gap-2">
                  <Button variant="outline" size="sm" onClick={() => setAutoRotate(!autoRotate)}>
                    <RotateCw className="w-4 h-4 mr-1" />
                    Auto Rotate
                  </Button>
                  <Button variant="outline" size="sm" onClick={() => setShowShadows(!showShadows)}>
                    <Settings className="w-4 h-4 mr-1" />
                    Shadows
                  </Button>
                </div>
              </div>

              {/* Telemetry */}
              {selectedRobot && telemetryData[selectedRobot] && (
                <div className="pt-4 border-t border-border/50">
                  <div className="text-sm font-semibold mb-3">Robot Telemetry</div>
                  <div className="space-y-2 text-xs">
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground">Position</span>
                      <span className="font-mono text-primary">
                        X: {telemetryData[selectedRobot].position.x.toFixed(2)} 
                        Y: {telemetryData[selectedRobot].position.y.toFixed(2)} 
                        Z: {telemetryData[selectedRobot].position.z.toFixed(2)}
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-muted-foreground">Velocity</span>
                      <span className="font-mono text-secondary">
                        {Math.sqrt(
                          Math.pow(telemetryData[selectedRobot].velocity.x, 2) +
                          Math.pow(telemetryData[selectedRobot].velocity.y, 2) +
                          Math.pow(telemetryData[selectedRobot].velocity.z, 2)
                        ).toFixed(2)} m/s
                      </span>
                    </div>
                  </div>
                </div>
              )}
            </CardContent>
          </Card>
        </Html>
      </Canvas>
    </div>
  )
}
