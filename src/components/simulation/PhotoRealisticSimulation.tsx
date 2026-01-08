'use client'

import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { 
  OrbitControls, 
  Environment, 
  PerspectiveCamera,
  ContactShadows,
  AccumulativeShadows,
  RandomizedLight,
  Center,
  Float
} from '@react-three/drei'
import { Suspense, useRef, useState, useEffect } from 'react'
import * as THREE from 'three'

// Robot Model Component - Procedurally generated humanoid robot
function RobotModel({ 
  selectedRobot, 
  isRunning, 
  speed,
  jointAngles 
}: { 
  selectedRobot: string
  isRunning: boolean
  speed: number
  jointAngles: number[]
}) {
  const groupRef = useRef<THREE.Group>(null)
  const [hovered, setHovered] = useState(false)

  // Animate robot based on state
  useFrame((state) => {
    if (groupRef.current && isRunning) {
      // Subtle idle animation
      groupRef.current.rotation.y = Math.sin(state.clock.elapsedTime * 0.5) * 0.05
      groupRef.current.position.y = Math.sin(state.clock.elapsedTime * 2) * 0.02
    }
  })

  // Robot colors based on selection - Professional Rovyn brand colors
  const robotColors: Record<string, { primary: string; secondary: string; accent: string }> = {
    'unitree-g1': { primary: '#0B0F19', secondary: '#1a1f2e', accent: '#2E7DFF' },
    'boston-atlas': { primary: '#0B0F19', secondary: '#1a1f2e', accent: '#00C2A8' },
    'tesla-optimus': { primary: '#0B0F19', secondary: '#1a1f2e', accent: '#FFD166' },
    'agility-digit': { primary: '#0B0F19', secondary: '#1a1f2e', accent: '#2E7DFF' },
  }

  const colors = robotColors[selectedRobot] || robotColors['unitree-g1']

  return (
    <group ref={groupRef} onPointerOver={() => setHovered(true)} onPointerOut={() => setHovered(false)}>
      {/* Torso */}
      <mesh position={[0, 1.2, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.4, 0.6, 0.3]} />
        <meshStandardMaterial 
          color={colors.primary} 
          metalness={0.8} 
          roughness={0.2}
          emissive={hovered ? colors.accent : '#000000'}
          emissiveIntensity={hovered ? 0.3 : 0}
        />
      </mesh>

      {/* Head */}
      <mesh position={[0, 1.7, 0]} castShadow>
        <boxGeometry args={[0.25, 0.25, 0.25]} />
        <meshStandardMaterial 
          color={colors.secondary} 
          metalness={0.9} 
          roughness={0.1}
        />
      </mesh>
      
      {/* Eyes */}
      <mesh position={[-0.08, 1.7, 0.13]} castShadow>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial color={colors.accent} emissive={colors.accent} emissiveIntensity={0.5} />
      </mesh>
      <mesh position={[0.08, 1.7, 0.13]} castShadow>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial color={colors.accent} emissive={colors.accent} emissiveIntensity={0.5} />
      </mesh>

      {/* Left Arm */}
      <group position={[-0.25, 1.2, 0]}>
        <mesh rotation={[0, 0, jointAngles[0] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial color={colors.primary} metalness={0.8} roughness={0.2} />
        </mesh>
        <mesh position={[0, -0.25, 0]} rotation={[0, 0, jointAngles[1] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.1, 0.3, 0.1]} />
          <meshStandardMaterial color={colors.secondary} metalness={0.8} roughness={0.2} />
        </mesh>
      </group>

      {/* Right Arm */}
      <group position={[0.25, 1.2, 0]}>
        <mesh rotation={[0, 0, -jointAngles[0] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial color={colors.primary} metalness={0.8} roughness={0.2} />
        </mesh>
        <mesh position={[0, -0.25, 0]} rotation={[0, 0, -jointAngles[1] * Math.PI / 180]} castShadow>
          <boxGeometry args={[0.1, 0.3, 0.1]} />
          <meshStandardMaterial color={colors.secondary} metalness={0.8} roughness={0.2} />
        </mesh>
      </group>

      {/* Left Leg */}
      <group position={[-0.1, 0.6, 0]}>
        <mesh rotation={[jointAngles[2] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.15, 0.5, 0.15]} />
          <meshStandardMaterial color={colors.primary} metalness={0.8} roughness={0.2} />
        </mesh>
        <mesh position={[0, -0.3, 0]} rotation={[jointAngles[3] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial color={colors.secondary} metalness={0.8} roughness={0.2} />
        </mesh>
        <mesh position={[0, -0.55, 0]} castShadow>
          <boxGeometry args={[0.15, 0.1, 0.25]} />
          <meshStandardMaterial color={colors.accent} metalness={0.9} roughness={0.1} />
        </mesh>
      </group>

      {/* Right Leg */}
      <group position={[0.1, 0.6, 0]}>
        <mesh rotation={[-jointAngles[2] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.15, 0.5, 0.15]} />
          <meshStandardMaterial color={colors.primary} metalness={0.8} roughness={0.2} />
        </mesh>
        <mesh position={[0, -0.3, 0]} rotation={[-jointAngles[3] * Math.PI / 180, 0, 0]} castShadow>
          <boxGeometry args={[0.12, 0.4, 0.12]} />
          <meshStandardMaterial color={colors.secondary} metalness={0.8} roughness={0.2} />
        </mesh>
        <mesh position={[0, -0.55, 0]} castShadow>
          <boxGeometry args={[0.15, 0.1, 0.25]} />
          <meshStandardMaterial color={colors.accent} metalness={0.9} roughness={0.1} />
        </mesh>
      </group>
    </group>
  )
}

// Premium Lab Environment
function LabEnvironment() {
  return (
    <>
      {/* Floor - Premium polished concrete */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
        <planeGeometry args={[20, 20]} />
        <meshStandardMaterial 
          color="#2a2a2a" 
          roughness={0.3} 
          metalness={0.1}
        />
      </mesh>

      {/* Grid pattern on floor */}
      <gridHelper args={[20, 20, '#444444', '#222222']} position={[0, 0.01, 0]} />

      {/* Walls - Modern lab setting */}
      <mesh rotation={[0, Math.PI / 2, 0]} position={[-10, 5, 0]} receiveShadow>
        <planeGeometry args={[10, 10]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.4} metalness={0.1} />
      </mesh>
      <mesh rotation={[0, -Math.PI / 2, 0]} position={[10, 5, 0]} receiveShadow>
        <planeGeometry args={[10, 10]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.4} metalness={0.1} />
      </mesh>
      <mesh position={[0, 5, -10]} receiveShadow>
        <planeGeometry args={[20, 10]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.4} metalness={0.1} />
      </mesh>

      {/* Ceiling lights - Studio lighting */}
      {[-5, 0, 5].map((x, i) => (
        <group key={i} position={[x, 4.5, -3]}>
          <mesh castShadow>
            <cylinderGeometry args={[0.3, 0.3, 0.1]} />
            <meshStandardMaterial color="#ffffff" emissive="#ffffff" emissiveIntensity={2} />
          </mesh>
          <pointLight position={[0, -0.2, 0]} intensity={2} distance={10} decay={2} castShadow />
        </group>
      ))}

      {/* Equipment racks */}
      {[-6, 6].map((x, i) => (
        <group key={i} position={[x, 1, -8]}>
          <mesh castShadow receiveShadow>
            <boxGeometry args={[0.3, 2, 0.3]} />
            <meshStandardMaterial color="#333333" metalness={0.7} roughness={0.3} />
          </mesh>
          <mesh position={[0, 1, 0]} castShadow receiveShadow>
            <boxGeometry args={[0.3, 0.05, 0.5]} />
            <meshStandardMaterial color="#444444" metalness={0.7} roughness={0.3} />
          </mesh>
        </group>
      ))}

      {/* Monitor screens */}
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
              emissiveIntensity={0.5}
            />
          </mesh>
        </group>
      ))}
    </>
  )
}

// Main Scene Component
function Scene({ 
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

  // Camera animation
  useFrame((state) => {
    if (cameraMode === 'auto' && camera instanceof THREE.PerspectiveCamera) {
      const time = state.clock.elapsedTime * speed
      camera.position.x = Math.sin(time * 0.5) * 5
      camera.position.z = Math.cos(time * 0.5) * 5
      camera.position.y = 3 + Math.sin(time * 0.25) * 1
      camera.lookAt(0, 1, 0)
    }
  })

  return (
    <>
      {/* Premium lighting setup */}
      <ambientLight intensity={0.3} />
      <directionalLight 
        position={[5, 10, 5]} 
        intensity={1} 
        castShadow 
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-far={50}
        shadow-camera-left={-10}
        shadow-camera-right={10}
        shadow-camera-top={10}
        shadow-camera-bottom={-10}
      />
      <pointLight position={[-5, 5, 5]} intensity={0.5} color="#2E7DFF" />
      <pointLight position={[5, 5, -5]} intensity={0.5} color="#00C2A8" />

      {/* Environment - HDRI or studio environment */}
      <Environment preset="studio" />

      {/* Lab environment */}
      <LabEnvironment />

      {/* Robot with realistic materials */}
      <Center>
        <Float speed={isRunning ? 1.5 * speed : 0} rotationIntensity={0.5} floatIntensity={0.5}>
          <RobotModel 
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={speed}
            jointAngles={jointAngles}
          />
        </Float>
      </Center>

      {/* Contact shadows for realism */}
      <ContactShadows 
        position={[0, 0.01, 0]} 
        opacity={0.4} 
        scale={10} 
        blur={2} 
        far={10} 
        resolution={512}
      />

      {/* Accumulative shadows for soft lighting */}
      <AccumulativeShadows
        position={[0, 0.01, 0]}
        frames={60}
        alphaTest={0.85}
        scale={10}
        color="#000000"
        opacity={0.6}
      >
        <RandomizedLight
          amount={8}
          radius={5}
          position={[5, 5, -5]}
          bias={0.001}
        />
      </AccumulativeShadows>
    </>
  )
}

// Main Component
export default function PhotoRealisticSimulation({
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
  return (
    <div className="w-full h-full">
      <Canvas
        shadows
        gl={{ 
          antialias: true, 
          toneMapping: THREE.ACESFilmicToneMapping,
          toneMappingExposure: 1.2,
          outputColorSpace: THREE.SRGBColorSpace
        }}
        dpr={[1, 2]}
      >
        <Suspense fallback={null}>
          <PerspectiveCamera makeDefault position={[5, 3, 5]} fov={50} />
          <Scene 
            selectedRobot={selectedRobot}
            isRunning={isRunning}
            speed={speed}
            cameraMode={cameraMode}
            jointAngles={jointAngles}
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
            />
          )}
        </Suspense>
      </Canvas>
    </div>
  )
}