'use client'

import { useRef, useMemo } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { Line } from '@react-three/drei'

// LiDAR Point Cloud Visualization
export function LiDARVisualization({ 
  position, 
  range = 5, 
  points = 360,
  enabled = true 
}: { 
  position: THREE.Vector3
  range?: number
  points?: number
  enabled?: boolean
}) {
  const lidarPointsRef = useRef<THREE.Vector3[]>([])
  const meshRef = useRef<THREE.Points>(null)

  const lidarGeometry = useMemo(() => {
    const geometry = new THREE.BufferGeometry()
    const positions = new Float32Array(points * 3)
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    return geometry
  }, [points])

  useFrame((state) => {
    if (!enabled || !meshRef.current) return

    const time = state.clock.elapsedTime
    const newPoints: THREE.Vector3[] = []

    // Simulate LiDAR scanning
    for (let i = 0; i < points; i++) {
      const angle = (i / points) * Math.PI * 2 + time * 0.5
      const distance = range * (0.7 + Math.random() * 0.3) // Simulated obstacle detection
      
      const x = Math.cos(angle) * distance
      const z = Math.sin(angle) * distance
      newPoints.push(new THREE.Vector3(x, 0.1, z))
    }

    lidarPointsRef.current = newPoints

    // Update geometry
    const positions = lidarGeometry.attributes.position.array as Float32Array
    newPoints.forEach((point, i) => {
      positions[i * 3] = point.x
      positions[i * 3 + 1] = point.y
      positions[i * 3 + 2] = point.z
    })
    lidarGeometry.attributes.position.needsUpdate = true
  })

  if (!enabled) return null

  return (
    <group position={position}>
      <points ref={meshRef} geometry={lidarGeometry}>
        <pointsMaterial size={0.05} color="#00ff00" transparent opacity={0.6} />
      </points>
      {/* LiDAR rays */}
      {lidarPointsRef.current.slice(0, 36).map((point, i) => (
        <Line
          key={i}
          points={[new THREE.Vector3(0, 0.1, 0), point]}
          color="#00ff00"
          lineWidth={1}
          transparent
          opacity={0.3}
        />
      ))}
    </group>
  )
}

// Depth Camera Visualization
export function DepthCameraVisualization({ 
  position, 
  fov = 60,
  range = 3,
  enabled = true 
}: { 
  position: THREE.Vector3
  fov?: number
  range?: number
  enabled?: boolean
}) {
  const depthMeshRef = useRef<THREE.Mesh>(null)

  useFrame((state) => {
    if (!enabled || !depthMeshRef.current) return

    const time = state.clock.elapsedTime
    const material = depthMeshRef.current.material as THREE.MeshStandardMaterial
    
    // Animate depth visualization
    material.emissiveIntensity = 0.3 + Math.sin(time * 2) * 0.2
  })

  if (!enabled) return null

  return (
    <group position={position}>
      {/* Depth camera frustum */}
      <mesh ref={depthMeshRef}>
        <coneGeometry args={[range * 0.5, range, 32, 1, true]} />
        <meshStandardMaterial 
          color="#0066ff" 
          transparent 
          opacity={0.2}
          side={THREE.DoubleSide}
          emissive="#0066ff"
          emissiveIntensity={0.3}
        />
      </mesh>
      {/* Camera indicator */}
      <mesh position={[0, 0, -0.1]}>
        <boxGeometry args={[0.1, 0.1, 0.05]} />
        <meshStandardMaterial color="#0066ff" emissive="#0066ff" emissiveIntensity={0.5} />
      </mesh>
    </group>
  )
}

// IMU Visualization (orientation indicator)
export function IMUVisualization({ 
  position,
  rotation,
  enabled = true 
}: { 
  position: THREE.Vector3
  rotation: THREE.Euler
  enabled?: boolean
}) {
  if (!enabled) return null

  return (
    <group position={position} rotation={rotation}>
      {/* X-axis (red) */}
      <arrowHelper args={[new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.3, 0xff0000]} />
      {/* Y-axis (green) */}
      <arrowHelper args={[new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.3, 0x00ff00]} />
      {/* Z-axis (blue) */}
      <arrowHelper args={[new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.3, 0x0000ff]} />
    </group>
  )
}

// Collision Detection Visualization
export function CollisionVisualization({ 
  position,
  radius = 0.5,
  colliding = false,
  enabled = true 
}: { 
  position: THREE.Vector3
  radius?: number
  colliding?: boolean
  enabled?: boolean
}) {
  if (!enabled) return null

  return (
    <group position={position}>
      <mesh>
        <sphereGeometry args={[radius, 16, 16]} />
        <meshStandardMaterial 
          color={colliding ? "#ff0000" : "#ffff00"}
          transparent
          opacity={0.2}
          wireframe
        />
      </mesh>
      {colliding && (
        <mesh>
          <sphereGeometry args={[radius * 1.1, 16, 16]} />
          <meshStandardMaterial 
            color="#ff0000"
            transparent
            opacity={0.1}
          />
        </mesh>
      )}
    </group>
  )
}