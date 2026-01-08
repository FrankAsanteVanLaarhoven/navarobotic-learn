'use client'

import { useMemo, useRef, useState } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { Line } from '@react-three/drei'

// A* Path Planning Algorithm
class AStarPathPlanner {
  private grid: number[][]
  private rows: number
  private cols: number

  constructor(gridSize: number = 20) {
    this.rows = gridSize
    this.cols = gridSize
    this.grid = Array(this.rows).fill(0).map(() => Array(this.cols).fill(0))
  }

  // Heuristic function (Manhattan distance)
  private heuristic(a: { x: number; y: number }, b: { x: number; y: number }): number {
    return Math.abs(a.x - b.x) + Math.abs(a.y - b.y)
  }

  // Find path using A* algorithm
  findPath(start: { x: number; y: number }, end: { x: number; y: number }): THREE.Vector3[] {
    const openSet: Array<{ x: number; y: number; f: number; g: number; h: number; cameFrom?: any }> = [start]
    const closedSet: Array<{ x: number; y: number }> = []
    const gScore: Map<string, number> = new Map()
    const fScore: Map<string, number> = new Map()

    gScore.set(`${start.x},${start.y}`, 0)
    fScore.set(`${start.x},${start.y}`, this.heuristic(start, end))

    while (openSet.length > 0) {
      // Find node with lowest f score
      let current = openSet[0]
      let currentIndex = 0
      for (let i = 1; i < openSet.length; i++) {
        const f = fScore.get(`${openSet[i].x},${openSet[i].y}`) || Infinity
        const currentF = fScore.get(`${current.x},${current.y}`) || Infinity
        if (f < currentF) {
          current = openSet[i]
          currentIndex = i
        }
      }

      openSet.splice(currentIndex, 1)
      closedSet.push(current)

      // Reconstruct path if reached goal
      if (current.x === end.x && current.y === end.y) {
        const path: THREE.Vector3[] = []
        let node: any = current
        while (node) {
          path.unshift(new THREE.Vector3(
            (node.x - this.cols / 2) * 0.5,
            0.1,
            (node.y - this.rows / 2) * 0.5
          ))
          node = node.cameFrom
        }
        return path
      }

      // Check neighbors
      const neighbors = [
        { x: current.x + 1, y: current.y },
        { x: current.x - 1, y: current.y },
        { x: current.x, y: current.y + 1 },
        { x: current.x, y: current.y - 1 },
      ]

      for (const neighbor of neighbors) {
        if (
          neighbor.x < 0 || neighbor.x >= this.cols ||
          neighbor.y < 0 || neighbor.y >= this.rows ||
          this.grid[neighbor.y][neighbor.x] === 1 || // Obstacle
          closedSet.some(n => n.x === neighbor.x && n.y === neighbor.y)
        ) {
          continue
        }

        const tentativeG = (gScore.get(`${current.x},${current.y}`) || 0) + 1

        if (!openSet.some(n => n.x === neighbor.x && n.y === neighbor.y)) {
          openSet.push(neighbor)
        } else if (tentativeG >= (gScore.get(`${neighbor.x},${neighbor.y}`) || Infinity)) {
          continue
        }

        neighbor.cameFrom = current
        gScore.set(`${neighbor.x},${neighbor.y}`, tentativeG)
        fScore.set(`${neighbor.x},${neighbor.y}`, tentativeG + this.heuristic(neighbor, end))
      }
    }

    return [] // No path found
  }

  setObstacle(x: number, y: number) {
    if (x >= 0 && x < this.cols && y >= 0 && y < this.rows) {
      this.grid[y][x] = 1
    }
  }
}

// Path Planning Visualization Component
export function PathPlanningVisualization({ 
  start,
  end,
  enabled = true 
}: { 
  start: THREE.Vector3
  end: THREE.Vector3
  enabled?: boolean
}) {
  const plannerRef = useRef(new AStarPathPlanner(20))
  const pathRef = useRef<THREE.Vector3[]>([])
  const [path, setPath] = useState<THREE.Vector3[]>([])

  // Initialize obstacles
  useMemo(() => {
    const planner = plannerRef.current
    // Add some random obstacles
    for (let i = 0; i < 10; i++) {
      planner.setObstacle(
        Math.floor(Math.random() * 20),
        Math.floor(Math.random() * 20)
      )
    }
  }, [])

  useFrame(() => {
    if (!enabled) return

    const planner = plannerRef.current
    const startGrid = {
      x: Math.floor(start.x / 0.5 + 10),
      y: Math.floor(start.z / 0.5 + 10)
    }
    const endGrid = {
      x: Math.floor(end.x / 0.5 + 10),
      y: Math.floor(end.z / 0.5 + 10)
    }

    const newPath = planner.findPath(startGrid, endGrid)
    if (newPath.length > 0 && JSON.stringify(newPath) !== JSON.stringify(pathRef.current)) {
      pathRef.current = newPath
      setPath(newPath)
    }
  })

  if (!enabled || path.length === 0) return null

  return (
    <>
      <Line
        points={path}
        color="#ff00ff"
        lineWidth={3}
        transparent
        opacity={0.8}
      />
      {/* Path nodes */}
      {path.map((point, i) => (
        <mesh key={i} position={point}>
          <sphereGeometry args={[0.05, 8, 8]} />
          <meshStandardMaterial color="#ff00ff" emissive="#ff00ff" emissiveIntensity={0.5} />
        </mesh>
      ))}
      {/* Start marker */}
      <mesh position={start}>
        <cylinderGeometry args={[0.1, 0.1, 0.2]} />
        <meshStandardMaterial color="#00ff00" />
      </mesh>
      {/* End marker */}
      <mesh position={end}>
        <cylinderGeometry args={[0.1, 0.1, 0.2]} />
        <meshStandardMaterial color="#ff0000" />
      </mesh>
    </>
  )
}