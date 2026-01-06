'use client'

import { Canvas, useFrame, useThree, extend } from '@react-three/fiber'
import { 
  OrbitControls, 
  Environment, 
  PerspectiveCamera,
  ContactShadows,
  Center,
  Float,
  Grid,
  MeshReflectorMaterial,
  useTexture,
  Line
} from '@react-three/drei'
import { 
  EffectComposer, 
  Bloom, 
  ChromaticAberration,
  DepthOfField,
  SSAO,
  Vignette,
  ToneMapping,
  BrightnessContrast
} from '@react-three/postprocessing'
import { Suspense, useRef, useState, useEffect, useMemo } from 'react'
import * as THREE from 'three'
import { 
  LiDARVisualization, 
  DepthCameraVisualization, 
  IMUVisualization,
  CollisionVisualization 
} from './SensorVisualization'
import { PathPlanningVisualization } from './PathPlanning'

// Advanced IK System for realistic robot movement
class IKSystem {
  private target: THREE.Vector3
  private chain: THREE.Vector3[]
  
  constructor() {
    this.target = new THREE.Vector3()
    this.chain = []
  }

  solveIK(base: THREE.Vector3, target: THREE.Vector3, chainLength: number): number[] {
    const angles: number[] = []
    const maxIterations = 100
    const tolerance = 0.01
    
    for (let i = 0; i < maxIterations; i++) {
      // CCD (Cyclic Coordinate Descent) IK solver
      const endEffector = this.forwardKinematics(base, angles, chainLength)
      const error = target.clone().sub(endEffector).length()
      
      if (error < tolerance) break
      
      // Update angles to minimize error
      for (let j = angles.length - 1; j >= 0; j--) {
        const toEnd = endEffector.clone().sub(base)
        const toTarget = target.clone().sub(base)
        const angle = toEnd.angleTo(toTarget)
        angles[j] = (angles[j] || 0) + angle * 0.1
      }
    }
    
    return angles
  }

  private forwardKinematics(base: THREE.Vector3, angles: number[], length: number): THREE.Vector3 {
    let pos = base.clone()
    let angle = 0
    
    for (let i = 0; i < angles.length; i++) {
      angle += angles[i]
      pos.x += Math.cos(angle) * length
      pos.y += Math.sin(angle) * length
    }
    
    return pos
  }
}

// Trajectory visualization
function TrajectoryPath({ points }: { points: THREE.Vector3[] }) {
  const curvePoints = useMemo(() => {
    if (points.length < 2) return []
    const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal')
    return curve.getPoints(100)
  }, [points])

  if (curvePoints.length === 0) return null

  return (
    <Line
      points={curvePoints}
      color="#00d4ff"
      lineWidth={2}
      transparent={true}
      opacity={0.6}
    />
  )
}

// Force vector visualization
function ForceVector({ 
  position, 
  force, 
  color = "#ff0000" 
}: { 
  position: THREE.Vector3
  force: THREE.Vector3
  color?: string
}) {
  const arrowLength = force.length() * 0.1
  const direction = force.clone().normalize()
  const end = position.clone().add(direction.multiplyScalar(arrowLength))

  const linePoints = useMemo(() => [position, end], [position, end])
  
  return (
    <group>
      <Line
        points={linePoints}
        color={color}
        lineWidth={3}
      />
      <mesh position={end}>
        <coneGeometry args={[0.05, 0.1, 8]} />
        <meshStandardMaterial color={color} />
      </mesh>
    </group>
  )
}

// Enhanced Robot Model with IK
function AdvancedRobotModel({ 
  selectedRobot, 
  isRunning, 
  speed,
  jointAngles,
  targetPosition
}: { 
  selectedRobot: string
  isRunning: boolean
  speed: number
  jointAngles: number[]
  targetPosition?: THREE.Vector3
}) {
  const groupRef = useRef<THREE.Group>(null)
  const leftArmRef = useRef<THREE.Group>(null)
  const rightArmRef = useRef<THREE.Group>(null)
  const leftLegRef = useRef<THREE.Group>(null)
  const rightLegRef = useRef<THREE.Group>(null)
  const [hovered, setHovered] = useState(false)
  const ikSystem = useMemo(() => new IKSystem(), [])

  // Robot colors with enhanced materials
  const robotColors: Record<string, { primary: string; secondary: string; accent: string; metalness: number; roughness: number }> = {
    'unitree-g1': { 
      primary: '#1a1a2e', 
      secondary: '#16213e', 
      accent: '#00d4ff',
      metalness: 0.9,
      roughness: 0.1
    },
    'boston-atlas': { 
      primary: '#2d1b1b', 
      secondary: '#3d2525', 
      accent: '#ff6b6b',
      metalness: 0.85,
      roughness: 0.15
    },
    'tesla-optimus': { 
      primary: '#1a2e1a', 
      secondary: '#253d25', 
      accent: '#a3ffb3',
      metalness: 0.8,
      roughness: 0.2
    },
    'agility-digit': { 
      primary: '#2e2a1b', 
      secondary: '#3d3525', 
      accent: '#ffa500',
      metalness: 0.9,
      roughness: 0.1
    },
  }

  const colors = robotColors[selectedRobot] || robotColors['unitree-g1']

  // Advanced animation with IK
  useFrame((state) => {
    if (groupRef.current && isRunning) {
      const time = state.clock.elapsedTime * speed
      
      // Breathing/idle animation
      groupRef.current.rotation.y = Math.sin(time * 0.5) * 0.05
      groupRef.current.position.y = Math.sin(time * 2) * 0.02
      
      // IK-based arm movement
      if (leftArmRef.current && targetPosition) {
        const basePos = new THREE.Vector3(-0.25, 1.2, 0)
        const ikAngles = ikSystem.solveIK(basePos, targetPosition, 0.7)
        if (ikAngles.length > 0) {
          leftArmRef.current.rotation.z = ikAngles[0]
        }
      }
      
      // Walking animation
      if (leftLegRef.current && rightLegRef.current) {
        leftLegRef.current.rotation.x = Math.sin(time * 2) * 0.3
        rightLegRef.current.rotation.x = -Math.sin(time * 2) * 0.3
      }
    }
  })

  return (
    <group ref={groupRef} onPointerOver={() => setHovered(true)} onPointerOut={() => setHovered(false)}>
      {/* Enhanced Torso with better geometry */}
      <mesh position={[0, 1.2, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.4, 0.6, 0.3]} />
        <meshStandardMaterial 
          color={colors.primary} 
          metalness={colors.metalness} 
          roughness={colors.roughness}
          emissive={hovered ? colors.accent : '#000000'}
          emissiveIntensity={hovered ? 0.5 : 0}
          envMapIntensity={1.5}
        />
      </mesh>

      {/* Enhanced Head */}
      <mesh position={[0, 1.7, 0]} castShadow>
        <boxGeometry args={[0.25, 0.25, 0.25]} />
        <meshStandardMaterial 
          color={colors.secondary} 
          metalness={colors.metalness} 
          roughness={colors.roughness}
          envMapIntensity={1.5}
        />
      </mesh>
      
      {/* Animated Eyes with glow */}
      <mesh position={[-0.08, 1.7, 0.13]} castShadow>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial 
          color={colors.accent} 
          emissive={colors.accent} 
          emissiveIntensity={isRunning ? 1 : 0.3}
        />
      </mesh>
      <mesh position={[0.08, 1.7, 0.13]} castShadow>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial 
          color={colors.accent} 
          emissive={colors.accent} 
          emissiveIntensity={isRunning ? 1 : 0.3}
        />
      </mesh>

      {/* Left Arm with IK */}
      <group ref={leftArmRef} position={[-0.25, 1.2, 0]}>
        <mesh rotation={[0, 0, jointAngles[0] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial 
            color={colors.primary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
        <mesh position={[0, -0.25, 0]} rotation={[0, 0, jointAngles[1] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.1, 0.3, 0.1]} />
          <meshStandardMaterial 
            color={colors.secondary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
      </group>

      {/* Right Arm */}
      <group ref={rightArmRef} position={[0.25, 1.2, 0]}>
        <mesh rotation={[0, 0, -jointAngles[0] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial 
            color={colors.primary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
        <mesh position={[0, -0.25, 0]} rotation={[0, 0, -jointAngles[1] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.1, 0.3, 0.1]} />
          <meshStandardMaterial 
            color={colors.secondary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
      </group>

      {/* Left Leg with walking animation */}
      <group ref={leftLegRef} position={[-0.1, 0.6, 0]}>
        <mesh rotation={[jointAngles[2] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.15, 0.5, 0.15]} />
          <meshStandardMaterial 
            color={colors.primary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
        <mesh position={[0, -0.3, 0]} rotation={[jointAngles[3] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial 
            color={colors.secondary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
        <mesh position={[0, -0.55, 0]} castShadow>
          <boxGeometry args={[0.15, 0.1, 0.25]} />
          <meshStandardMaterial 
            color={colors.accent} 
            metalness={0.95} 
            roughness={0.05}
            emissive={colors.accent}
            emissiveIntensity={0.2}
          />
        </mesh>
      </group>

      {/* Right Leg */}
      <group ref={rightLegRef} position={[0.1, 0.6, 0]}>
        <mesh rotation={[-jointAngles[2] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.15, 0.5, 0.15]} />
          <meshStandardMaterial 
            color={colors.primary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
        <mesh position={[0, -0.3, 0]} rotation={[-jointAngles[3] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial 
            color={colors.secondary} 
            metalness={colors.metalness} 
            roughness={colors.roughness}
            envMapIntensity={1.5}
          />
        </mesh>
        <mesh position={[0, -0.55, 0]} castShadow>
          <boxGeometry args={[0.15, 0.1, 0.25]} />
          <meshStandardMaterial 
            color={colors.accent} 
            metalness={0.95} 
            roughness={0.05}
            emissive={colors.accent}
            emissiveIntensity={0.2}
          />
        </mesh>
      </group>
    </group>
  )
}

// Premium Lab Environment with reflections
function PremiumLabEnvironment() {
  return (
    <>
      {/* Reflective floor */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
        <planeGeometry args={[20, 20]} />
        <MeshReflectorMaterial
          blur={[300, 100]}
          resolution={1024}
          mixBlur={1}
          mixStrength={50}
          roughness={0.5}
          depthScale={1.2}
          minDepthThreshold={0.4}
          maxDepthThreshold={1.4}
          color="#2a2a2a"
          metalness={0.8}
          mirror={0.5}
        />
      </mesh>

      {/* Grid helper */}
      <Grid
        renderOrder={-1}
        position={[0, 0.01, 0]}
        infiniteGrid
        cellSize={0.5}
        cellThickness={0.5}
        cellColor="#444444"
        sectionSize={2}
        sectionThickness={1}
        sectionColor="#666666"
        fadeDistance={15}
        fadeStrength={1}
      />

      {/* Enhanced walls */}
      <mesh rotation={[0, Math.PI / 2, 0]} position={[-10, 5, 0]} receiveShadow>
        <planeGeometry args={[10, 10]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.3} metalness={0.1} />
      </mesh>
      <mesh rotation={[0, -Math.PI / 2, 0]} position={[10, 5, 0]} receiveShadow>
        <planeGeometry args={[10, 10]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.3} metalness={0.1} />
      </mesh>
      <mesh position={[0, 5, -10]} receiveShadow>
        <planeGeometry args={[20, 10]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.3} metalness={0.1} />
      </mesh>

      {/* Enhanced studio lighting */}
      {[-5, 0, 5].map((x, i) => (
        <group key={i} position={[x, 4.5, -3]}>
          <mesh castShadow>
            <cylinderGeometry args={[0.3, 0.3, 0.1]} />
            <meshStandardMaterial 
              color="#ffffff" 
              emissive="#ffffff" 
              emissiveIntensity={3}
            />
          </mesh>
          <pointLight 
            position={[0, -0.2, 0]} 
            intensity={3} 
            distance={15} 
            decay={2} 
            castShadow
            shadow-mapSize-width={2048}
            shadow-mapSize-height={2048}
          />
        </group>
      ))}

      {/* Enhanced equipment */}
      {[-6, 6].map((x, i) => (
        <group key={i} position={[x, 1, -8]}>
          <mesh castShadow receiveShadow>
            <boxGeometry args={[0.3, 2, 0.3]} />
            <meshStandardMaterial color="#333333" metalness={0.8} roughness={0.2} />
          </mesh>
          <mesh position={[0, 1, 0]} castShadow receiveShadow>
            <boxGeometry args={[0.3, 0.05, 0.5]} />
            <meshStandardMaterial color="#444444" metalness={0.8} roughness={0.2} />
          </mesh>
        </group>
      ))}

      {/* Enhanced monitors */}
      {[-3, 3].map((x, i) => (
        <group key={i} position={[x, 1.5, -9.8]}>
          <mesh castShadow receiveShadow>
            <boxGeometry args={[0.8, 0.5, 0.05]} />
            <meshStandardMaterial color="#000000" />
          </mesh>
          <mesh position={[0, 0, 0.03]} castShadow>
            <planeGeometry args={[0.75, 0.45]} />
            <meshStandardMaterial 
              color="#00ff00" 
              emissive="#00ff00" 
              emissiveIntensity={1}
            />
          </mesh>
        </group>
      ))}
    </>
  )
}

// Main Scene with Post-Processing
function AdvancedScene({ 
  selectedRobot, 
  isRunning, 
  speed, 
  cameraMode,
  jointAngles 
}: {
  selectedRobot: string
  isRunning: boolean
  speed: number
  cameraMode: string
  jointAngles: number[]
}) {
  const { camera } = useThree()
  const [trajectoryPoints, setTrajectoryPoints] = useState<THREE.Vector3[]>([])
  const targetPosition = useMemo(() => new THREE.Vector3(1, 1.5, 0.5), [])

  // Camera animation
  useFrame((state) => {
    if (cameraMode === 'auto' && camera instanceof THREE.PerspectiveCamera) {
      const time = state.clock.elapsedTime * speed
      camera.position.x = Math.sin(time * 0.5) * 5
      camera.position.z = Math.cos(time * 0.5) * 5
      camera.position.y = 3 + Math.sin(time * 0.25) * 1
      camera.lookAt(0, 1, 0)
    }
    
    // Update trajectory
    if (isRunning && state.clock.elapsedTime % 0.5 < 0.1) {
      setTrajectoryPoints(prev => {
        const newPoint = new THREE.Vector3(
          Math.sin(state.clock.elapsedTime) * 2,
          1,
          Math.cos(state.clock.elapsedTime) * 2
        )
        return [...prev.slice(-20), newPoint]
      })
    }
  })

  return (
    <>
      {/* Advanced lighting */}
      <ambientLight intensity={0.4} />
      <directionalLight 
        position={[5, 10, 5]} 
        intensity={1.5} 
        castShadow 
        shadow-mapSize-width={4096}
        shadow-mapSize-height={4096}
        shadow-camera-far={50}
        shadow-camera-left={-10}
        shadow-camera-right={10}
        shadow-camera-top={10}
        shadow-camera-bottom={-10}
        shadow-bias={-0.0001}
      />
      <pointLight position={[-5, 5, 5]} intensity={0.8} color="#00d4ff" />
      <pointLight position={[5, 5, -5]} intensity={0.8} color="#ff6b6b" />
      <spotLight
        position={[0, 8, 0]}
        angle={0.5}
        penumbra={0.5}
        intensity={1}
        castShadow
      />

      {/* HDRI Environment */}
      <Environment preset="studio" />

      {/* Premium environment */}
      <PremiumLabEnvironment />

      {/* Robot */}
      <Center>
        <Float speed={isRunning ? 1.5 * speed : 0} rotationIntensity={0.3} floatIntensity={0.3}>
          <AdvancedRobotModel 
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={speed}
            jointAngles={jointAngles}
            targetPosition={targetPosition}
          />
        </Float>
      </Center>

      {/* Trajectory visualization */}
      {trajectoryPoints.length > 1 && (
        <TrajectoryPath points={trajectoryPoints} />
      )}

      {/* Force vectors */}
      {isRunning && (
        <>
          <ForceVector 
            position={new THREE.Vector3(0, 1.2, 0)}
            force={new THREE.Vector3(0.1, 0.2, 0)}
            color="#00d4ff"
          />
        </>
      )}

      {/* Contact shadows */}
      <ContactShadows 
        position={[0, 0.01, 0]} 
        opacity={0.5} 
        scale={10} 
        blur={3} 
        far={10} 
        resolution={1024}
      />

      {/* Sensor Visualizations */}
      {showSensors && (
        <>
          <LiDARVisualization 
            position={new THREE.Vector3(0, 1.7, 0)}
            range={5}
            enabled={isRunning}
          />
          <DepthCameraVisualization 
            position={new THREE.Vector3(0, 1.6, 0.1)}
            enabled={isRunning}
          />
          <IMUVisualization 
            position={new THREE.Vector3(0, 1.5, 0)}
            rotation={new THREE.Euler(0, 0, 0)}
            enabled={isRunning}
          />
          <CollisionVisualization 
            position={new THREE.Vector3(0, 1, 0)}
            colliding={false}
            enabled={isRunning}
          />
        </>
      )}

      {/* Path Planning */}
      {showPathPlanning && (
        <PathPlanningVisualization 
          start={new THREE.Vector3(-2, 0.1, -2)}
          end={new THREE.Vector3(2, 0.1, 2)}
          enabled={isRunning}
        />
      )}
    </>
  )
}

// Main Component with Post-Processing
export default function AdvancedSimulation({
  selectedRobot,
  isRunning,
  speed,
  cameraMode,
  jointAngles,
  showSensors = true,
  showPathPlanning = true
}: {
  selectedRobot: string
  isRunning: boolean
  speed: number
  cameraMode: string
  jointAngles: number[]
  showSensors?: boolean
  showPathPlanning?: boolean
}) {
  return (
    <div className="w-full h-full">
      <Canvas
        shadows
        gl={{ 
          antialias: true, 
          toneMapping: THREE.ACESFilmicToneMapping,
          toneMappingExposure: 1.2,
          outputColorSpace: THREE.SRGBColorSpace,
          powerPreference: "high-performance"
        }}
        dpr={[1, 2]}
        performance={{ min: 0.5 }}
      >
        <Suspense fallback={null}>
          <PerspectiveCamera makeDefault position={[5, 3, 5]} fov={50} />
          <AdvancedScene 
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={speed}
            cameraMode={cameraMode}
            jointAngles={jointAngles}
            showSensors={showSensors}
            showPathPlanning={showPathPlanning}
          />
          {cameraMode === 'manual' && (
            <OrbitControls 
              enablePan={true}
              enableZoom={true}
              enableRotate={true}
              minDistance={3}
              maxDistance={15}
              minPolarAngle={0}
              maxPolarAngle={Math.PI / 2}
              dampingFactor={0.05}
            />
          )}
          
          {/* Post-Processing Effects */}
          <EffectComposer enableNormalPass>
            <Bloom 
              intensity={0.5} 
              luminanceThreshold={0.9}
              luminanceSmoothing={0.9}
              height={300}
            />
            <SSAO 
              samples={31}
              radius={5}
              intensity={30}
              luminanceInfluence={0.1}
              color="black"
            />
            <DepthOfField 
              focusDistance={0.02}
              focalLength={0.02}
              bokehScale={2}
              height={480}
            />
            <ChromaticAberration
              offset={[0.0005, 0.0012]}
            />
            <Vignette 
              eskil={false}
              offset={0.1}
              darkness={0.5}
            />
            <ToneMapping 
              resolution={256}
            />
            <BrightnessContrast 
              brightness={0.05}
              contrast={0.1}
            />
          </EffectComposer>
        </Suspense>
      </Canvas>
    </div>
  )
}