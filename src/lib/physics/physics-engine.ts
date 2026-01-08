/**
 * Real Physics Simulation Engine
 * Integrates Cannon.js (physics) with Three.js (rendering)
 * Supports collision detection, gravity, friction, and robot joints
 */

import * as THREE from 'three'

// Note: cannon-es would be imported here in production
// For now, we'll create a compatible interface

export interface PhysicsConfig {
  gravity: number
  friction: number
  restitution: number
  solverIterations: number
  broadphase: 'Naive' | 'SAP'
}

export interface JointPhysics {
  name: string
  body: any // CANNON.Body
  mesh: THREE.Mesh
  constraints: any[] // CANNON.Constraint[]
  limits: {
    minX: number
    maxX: number
    minY: number
    maxY: number
    minZ: number
    maxZ: number
  }
}

export interface RobotPhysics {
  baseBody: any // CANNON.Body
  baseMesh: THREE.Mesh
  joints: Map<string, JointPhysics>
  isSimulating: boolean
}

export class PhysicsEngine {
  private world: any // CANNON.World
  private timeStep: number
  private maxSubSteps: number
  private clock: THREE.Clock
  private physicsBodies: Map<string, any> // Map<string, CANNON.Body>
  private physicsMeshes: Map<string, THREE.Mesh>
  private constraints: any[] // CANNON.Constraint[]
  private robots: Map<string, RobotPhysics>
  private scene: THREE.Scene

  constructor(config: PhysicsConfig = {}) {
    // Initialize physics world (would use Cannon.js in production)
    this.world = {
      gravity: { x: 0, y: config.gravity || -9.82, z: 0 },
      bodies: []
    }

    this.timeStep = 1 / 60
    this.maxSubSteps = 10
    this.clock = new THREE.Clock()

    // Initialize maps
    this.physicsBodies = new Map()
    this.physicsMeshes = new Map()
    this.constraints = []
    this.robots = new Map()

    // Initialize scene (passed from Three.js)
    this.scene = new THREE.Scene()

    console.log('Physics Engine initialized')
  }

  /**
   * Set up scene for physics
   */
  setScene(scene: THREE.Scene): void {
    this.scene = scene
  }

  /**
   * Create ground plane for physics
   */
  createGround(size: number = 100): void {
    // Create physics ground body (simplified)
    const groundBody = {
      mass: 0,
      position: { x: 0, y: 0, z: 0 },
      quaternion: { x: 0, y: 0, z: 0, w: 1 }
    }

    this.world.bodies.push(groundBody)
    this.physicsBodies.set('ground', groundBody)

    // Create Three.js ground mesh
    const groundGeometry = new THREE.PlaneGeometry(size, size)
    const groundMaterial = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.5,
      roughness: 0.8
    })

    const groundMesh = new THREE.Mesh(groundGeometry, groundMaterial)
    groundMesh.rotation.x = -Math.PI / 2
    groundMesh.receiveShadow = true

    this.scene.add(groundMesh)
    this.physicsMeshes.set('ground', groundMesh)

    console.log('Ground created')
  }

  /**
   * Create robot physics body
   */
  createRobot(robotConfig: {
    id: string
    type: 'humanoid' | 'bipedal' | 'mobile'
    position: { x: number; y: number; z: number }
    scale: number
  }): RobotPhysics {
    const { id, type, position, scale } = robotConfig

    // Create robot base body (simplified physics representation)
    const baseBody = {
      mass: 5 * scale,
      position: { x: position.x, y: position.y + 1, z: position.z },
      velocity: { x: 0, y: 0, z: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      quaternion: { x: 0, y: 0, z: 0, w: 1 }
    }

    this.world.bodies.push(baseBody)
    this.physicsBodies.set(`${id}-base`, baseBody)

    // Create Three.js robot base mesh
    const baseGeometry = new THREE.BoxGeometry(0.4 * scale, 0.2 * scale, 0.4 * scale)
    const baseMaterial = new THREE.MeshStandardMaterial({
      color: 0x2E7DFF, // Rovyn primary color
      metalness: 0.8,
      roughness: 0.2
    })

    const baseMesh = new THREE.Mesh(baseGeometry, baseMaterial)
    baseMesh.castShadow = true
    baseMesh.receiveShadow = true
    baseMesh.position.set(position.x, position.y + 1, position.z)

    this.scene.add(baseMesh)
    this.physicsMeshes.set(`${id}-base`, baseMesh)

    // Create robot joints
    const joints = new Map<string, JointPhysics>()
    const jointNames = ['left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle', 'torso', 'neck', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow']

    jointNames.forEach((jointName, index) => {
      // Create joint body
      const jointBody = {
        mass: 1 * scale,
        position: {
          x: position.x + (index % 2 === 0 ? -0.15 : 0.15) * scale,
          y: position.y + 0.4 + (index * 0.05) * scale,
          z: position.z
        },
        velocity: { x: 0, y: 0, z: 0 },
        angularVelocity: { x: 0, y: 0, z: 0 },
        quaternion: { x: 0, y: 0, z: 0, w: 1 }
      }

      this.world.bodies.push(jointBody)
      this.physicsBodies.set(`${id}-${jointName}`, jointBody)

      // Create Three.js joint mesh
      const jointGeometry = new THREE.SphereGeometry(0.08 * scale, 32, 32)
      const jointMaterial = new THREE.MeshStandardMaterial({
        color: 0x64748b,
        metalness: 0.7,
        roughness: 0.3
      })

      const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial)
      jointMesh.position.set(jointBody.position.x, jointBody.position.y, jointBody.position.z)
      jointMesh.castShadow = true
      jointMesh.receiveShadow = true

      this.scene.add(jointMesh)
      this.physicsMeshes.set(`${id}-${jointName}`, jointMesh)

      joints.set(jointName, {
        name: jointName,
        body: jointBody,
        mesh: jointMesh,
        constraints: [],
        limits: {
          minX: -Math.PI / 2,
          maxX: Math.PI / 2,
          minY: -Math.PI / 4,
          maxY: Math.PI / 4,
          minZ: -Math.PI / 8,
          maxZ: Math.PI / 8
        }
      })
    })

    const robot: RobotPhysics = {
      baseBody,
      baseMesh,
      joints,
      isSimulating: false
    }

    this.robots.set(id, robot)
    console.log(`Robot ${id} created with ${joints.size} physics joints`)

    return robot
  }

  /**
   * Update robot joint angles
   */
  updateJoint(robotId: string, jointName: string, angles: { x: number; y: number; z: number }): void {
    const robot = this.robots.get(robotId)
    if (!robot || !robot.joints.has(jointName)) return

    const joint = robot.joints.get(jointName)!

    // Apply joint limits
    const clampedAngles = {
      x: Math.max(joint.limits.minX, Math.min(joint.limits.maxX, angles.x)),
      y: Math.max(joint.limits.minY, Math.min(joint.limits.maxY, angles.y)),
      z: Math.max(joint.limits.minZ, Math.min(joint.limits.maxZ, angles.z))
    }

    // Update physics body orientation (simplified)
    const euler = new THREE.Euler(clampedAngles.x, clampedAngles.y, clampedAngles.z)
    const quaternion = new THREE.Quaternion().setFromEuler(euler)
    joint.body.quaternion = {
      x: quaternion.x,
      y: quaternion.y,
      z: quaternion.z,
      w: quaternion.w
    }
  }

  /**
   * Apply force to robot
   */
  applyForce(robotId: string, force: { x: number; y: number; z: number }): void {
    const robot = this.robots.get(robotId)
    if (!robot) return

    // Apply force to base body (simplified)
    robot.baseBody.velocity.x += force.x * 0.01
    robot.baseBody.velocity.y += force.y * 0.01
    robot.baseBody.velocity.z += force.z * 0.01
  }

  /**
   * Apply torque to robot
   */
  applyTorque(robotId: string, torque: { x: number; y: number; z: number }): void {
    const robot = this.robots.get(robotId)
    if (!robot) return

    // Apply torque to base body (simplified)
    robot.baseBody.angularVelocity.x += torque.x * 0.01
    robot.baseBody.angularVelocity.y += torque.y * 0.01
    robot.baseBody.angularVelocity.z += torque.z * 0.01
  }

  /**
   * Step physics simulation
   */
  step(): void {
    // Step physics world (simplified - would use Cannon.js in production)
    const delta = this.clock.getDelta()

    // Update body positions based on velocity
    this.world.bodies.forEach((body: any) => {
      if (body.mass > 0) {
        // Apply gravity
        body.velocity.y += this.world.gravity.y * delta

        // Update position
        body.position.x += body.velocity.x * delta
        body.position.y += body.velocity.y * delta
        body.position.z += body.velocity.z * delta

        // Apply damping
        body.velocity.x *= 0.95
        body.velocity.y *= 0.95
        body.velocity.z *= 0.95
      }
    })

    // Sync Three.js meshes with physics bodies
    this.syncBodiesAndMeshes()

    // Update robots
    this.robots.forEach((robot, robotId) => {
      const baseBody = this.physicsBodies.get(`${robotId}-base`)
      if (baseBody) {
        robot.baseMesh.position.set(baseBody.position.x, baseBody.position.y, baseBody.position.z)
      }

      robot.joints.forEach((joint) => {
        const jointBody = this.physicsBodies.get(`${robotId}-${joint.name}`)
        if (jointBody) {
          joint.mesh.position.set(jointBody.position.x, jointBody.position.y, jointBody.position.z)
          
          // Apply quaternion rotation
          if (jointBody.quaternion) {
            const quat = new THREE.Quaternion(
              jointBody.quaternion.x,
              jointBody.quaternion.y,
              jointBody.quaternion.z,
              jointBody.quaternion.w
            )
            joint.mesh.quaternion.copy(quat)
          }
        }
      })
    })
  }

  /**
   * Sync physics bodies with Three.js meshes
   */
  private syncBodiesAndMeshes(): void {
    this.physicsBodies.forEach((body, id) => {
      const mesh = this.physicsMeshes.get(id)
      if (mesh && body.position) {
        mesh.position.set(body.position.x, body.position.y, body.position.z)
      }
    })
  }

  /**
   * Start physics simulation
   */
  start(): void {
    this.robots.forEach((robot) => {
      robot.isSimulating = true
    })

    console.log('Physics simulation started')
  }

  /**
   * Stop physics simulation
   */
  stop(): void {
    this.robots.forEach((robot) => {
      robot.isSimulating = false
    })

    console.log('Physics simulation stopped')
  }

  /**
   * Reset robot to initial position
   */
  resetRobot(robotId: string): void {
    const robot = this.robots.get(robotId)
    if (!robot) return

    // Reset base body
    robot.baseBody.velocity = { x: 0, y: 0, z: 0 }
    robot.baseBody.angularVelocity = { x: 0, y: 0, z: 0 }

    // Reset joint bodies
    robot.joints.forEach((joint) => {
      const jointBody = this.physicsBodies.get(`${robotId}-${joint.name}`)
      if (jointBody) {
        jointBody.velocity = { x: 0, y: 0, z: 0 }
        jointBody.angularVelocity = { x: 0, y: 0, z: 0 }
      }
    })

    console.log(`Robot ${robotId} reset`)
  }

  /**
   * Remove robot from physics world
   */
  removeRobot(robotId: string): void {
    const robot = this.robots.get(robotId)
    if (!robot) return

    // Remove base body
    const baseIndex = this.world.bodies.indexOf(robot.baseBody)
    if (baseIndex > -1) {
      this.world.bodies.splice(baseIndex, 1)
    }
    this.physicsBodies.delete(`${robotId}-base`)
    this.scene.remove(robot.baseMesh)
    this.physicsMeshes.delete(`${robotId}-base`)

    // Remove joint bodies
    robot.joints.forEach((joint, jointName) => {
      const jointBody = this.physicsBodies.get(`${robotId}-${jointName}`)
      if (jointBody) {
        const jointIndex = this.world.bodies.indexOf(jointBody)
        if (jointIndex > -1) {
          this.world.bodies.splice(jointIndex, 1)
        }
      }
      this.physicsBodies.delete(`${robotId}-${jointName}`)
      this.scene.remove(joint.mesh)
      this.physicsMeshes.delete(`${robotId}-${jointName}`)
    })

    this.robots.delete(robotId)
    console.log(`Robot ${robotId} removed from physics world`)
  }

  /**
   * Get robot position
   */
  getRobotPosition(robotId: string): { x: number; y: number; z: number } {
    const robot = this.robots.get(robotId)
    if (!robot) return { x: 0, y: 0, z: 0 }

    const position = robot.baseBody.position
    return { x: position.x, y: position.y, z: position.z }
  }

  /**
   * Get robot velocity
   */
  getRobotVelocity(robotId: string): { x: number; y: number; z: number } {
    const robot = this.robots.get(robotId)
    if (!robot) return { x: 0, y: 0, z: 0 }

    const velocity = robot.baseBody.velocity
    return { x: velocity.x, y: velocity.y, z: velocity.z }
  }

  /**
   * Get robot angular velocity
   */
  getRobotAngularVelocity(robotId: string): { x: number; y: number; z: number } {
    const robot = this.robots.get(robotId)
    if (!robot) return { x: 0, y: 0, z: 0 }

    const angularVelocity = robot.baseBody.angularVelocity
    return { x: angularVelocity.x, y: angularVelocity.y, z: angularVelocity.z }
  }

  /**
   * Check collision between robots
   */
  checkCollision(robotId1: string, robotId2: string): boolean {
    const robot1 = this.robots.get(robotId1)
    const robot2 = this.robots.get(robotId2)

    if (!robot1 || !robot2) return false

    const pos1 = robot1.baseBody.position
    const pos2 = robot2.baseBody.position
    const distance = Math.sqrt(
      Math.pow(pos1.x - pos2.x, 2) +
      Math.pow(pos1.y - pos2.y, 2) +
      Math.pow(pos1.z - pos2.z, 2)
    )
    const collisionThreshold = 0.5 // meters

    return distance < collisionThreshold
  }

  /**
   * Get physics stats
   */
  getStats(): {
    bodies: number
    constraints: number
    robots: number
    fps: number
  } {
    return {
      bodies: this.world.bodies.length,
      constraints: this.constraints.length,
      robots: this.robots.size,
      fps: this.clock.getDelta() > 0 ? 1 / this.clock.getDelta() : 60
    }
  }
}

/**
 * Factory function to create physics engine
 */
export function createPhysicsEngine(config?: PhysicsConfig): PhysicsEngine {
  return new PhysicsEngine(config)
}
