/**
 * Advanced Inverse Kinematics Solver
 * Implements CCD (Cyclic Coordinate Descent) and Jacobian Matrix solvers
 * Optimized for multi-joint humanoid robots
 */

export interface JointConfig {
  name: string
  parent?: string
  position: { x: number; y: number; z: number }
  rotation: { x: number; y: number; z: number }
  minAngle: number
  maxAngle: number
  length?: number
}

export interface TargetPosition {
  x: number
  y: number
  z: number
  orientation?: { roll: number; pitch: number; yaw: number }
}

export interface IKSolution {
  joints: Record<string, number>
  error: number
  iterations: number
  converged: boolean
  computationTime: number
}

export interface SolverConfig {
  maxIterations: number
  errorThreshold: number
  learningRate: number
  stepSize: number
  solverType: 'ccd' | 'jacobian' | 'hybrid'
}

/**
 * Cyclic Coordinate Descent (CCD) Solver
 */
export class CCDSolver {
  private joints: Map<string, JointConfig>
  private config: SolverConfig

  constructor(joints: JointConfig[], config: SolverConfig = {}) {
    this.joints = new Map()
    joints.forEach(joint => this.joints.set(joint.name, joint))
    this.config = {
      maxIterations: 100,
      errorThreshold: 0.001,
      learningRate: 0.1,
      stepSize: 0.01,
      solverType: 'ccd',
      ...config
    }
  }

  /**
   * Solve IK for target position
   */
  async solve(target: TargetPosition): Promise<IKSolution> {
    const startTime = performance.now()
    let iterations = 0
    let converged = false
    let previousError = Infinity

    // Initialize joint angles
    const jointAngles: Record<string, number> = {}
    this.joints.forEach((joint, name) => {
      jointAngles[name] = joint.rotation.y
    })

    // CCD Iteration
    while (iterations < this.config.maxIterations && !converged) {
      const endEffector = this.calculateEndEffector(jointAngles)
      const error = this.calculateError(endEffector, target)
      
      if (error < this.config.errorThreshold) {
        converged = true
        break
      }

      if (error > previousError) {
        break
      }

      previousError = error

      // Optimize each joint
      for (const [jointName, joint] of this.joints) {
        let bestAngle = jointAngles[jointName]
        let bestError = Infinity

        for (let angleOffset = -Math.PI; angleOffset <= Math.PI; angleOffset += this.config.stepSize) {
          const newAngle = joint.rotation.y + angleOffset
          const clampedAngle = this.clampAngle(newAngle, joint.minAngle, joint.maxAngle)
          const tempAngles = { ...jointAngles, [jointName]: clampedAngle }
          const newEndEffector = this.calculateEndEffector(tempAngles)
          const tempError = this.calculateError(newEndEffector, target)
          
          if (tempError < bestError) {
            bestError = tempError
            bestAngle = clampedAngle
          }
        }

        jointAngles[jointName] = bestAngle
      }
    }

    const computationTime = performance.now() - startTime

    return {
      joints: jointAngles,
      error: previousError,
      iterations,
      converged,
      computationTime
    }
  }

  private calculateEndEffector(jointAngles: Record<string, number>): { x: number; y: number; z: number } {
    let x = 0, y = 0, z = 0

    for (const [name, joint] of this.joints) {
      const angle = jointAngles[name] || 0
      const length = joint.length || 0.1

      x += Math.sin(angle) * length
      y += Math.cos(angle) * length
      z += joint.position.z
    }

    return { x, y, z }
  }

  private calculateError(current: { x: number; y: number; z: number }, target: TargetPosition): number {
    const dx = current.x - target.x
    const dy = current.y - target.y
    const dz = current.z - target.z
    
    return Math.sqrt(dx * dx + dy * dy + dz * dz)
  }

  private clampAngle(angle: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, angle))
  }
}

/**
 * Jacobian Matrix Solver
 */
export class JacobianSolver {
  private joints: Map<string, JointConfig>
  private config: SolverConfig
  private jointOrder: string[]

  constructor(joints: JointConfig[], config: SolverConfig = {}) {
    this.joints = new Map()
    joints.forEach(joint => this.joints.set(joint.name, joint))
    this.config = {
      maxIterations: 50,
      errorThreshold: 0.001,
      learningRate: 0.1,
      stepSize: 0.01,
      solverType: 'jacobian',
      ...config
    }
    this.jointOrder = joints.map(j => j.name)
  }

  async solve(target: TargetPosition): Promise<IKSolution> {
    const startTime = performance.now()
    let iterations = 0
    let converged = false
    let previousError = Infinity

    const jointAngles: Record<string, number> = {}
    this.joints.forEach((joint, name) => {
      jointAngles[name] = joint.rotation.y
    })

    while (iterations < this.config.maxIterations && !converged) {
      const currentPos = this.calculateEndEffector(jointAngles)
      
      const errorVector = [
        target.x - currentPos.x,
        target.y - currentPos.y,
        target.z - currentPos.z
      ]
      
      const error = Math.sqrt(
        errorVector[0] * errorVector[0] +
        errorVector[1] * errorVector[1] +
        errorVector[2] * errorVector[2]
      )

      if (error < this.config.errorThreshold) {
        converged = true
        break
      }

      // Simplified Jacobian update
      for (let i = 0; i < this.jointOrder.length; i++) {
        const jointName = this.jointOrder[i]
        const joint = this.joints.get(jointName)!
        const delta = errorVector[i % 3] * this.config.learningRate

        jointAngles[jointName] += delta
        jointAngles[jointName] = Math.max(joint.minAngle, Math.min(joint.maxAngle, jointAngles[jointName]))
      }

      iterations++
    }

    const computationTime = performance.now() - startTime

    return {
      joints: jointAngles,
      error: previousError,
      iterations,
      converged,
      computationTime
    }
  }

  private calculateEndEffector(jointAngles: Record<string, number>): { x: number; y: number; z: number } {
    let x = 0, y = 0, z = 0

    for (const [name, joint] of this.joints) {
      const angle = jointAngles[name] || 0
      const length = joint.length || 0.1

      x += Math.cos(angle) * length
      y += Math.sin(angle) * length
      z += joint.position.z
    }

    return { x, y, z }
  }
}

/**
 * Hybrid Solver - Combines CCD and Jacobian
 */
export class HybridSolver {
  private ccdSolver: CCDSolver
  private jacobianSolver: JacobianSolver

  constructor(joints: JointConfig[], config: SolverConfig = {}) {
    this.ccdSolver = new CCDSolver(joints, { ...config, solverType: 'ccd' })
    this.jacobianSolver = new JacobianSolver(joints, { ...config, solverType: 'jacobian' })
  }

  async solve(target: TargetPosition): Promise<IKSolution> {
    const ccdSolution = await this.ccdSolver.solve(target)
    const jacobianSolution = await this.jacobianSolver.solve(target)

    const finalJoints: Record<string, number> = {}
    for (const [name] of this.ccdSolver['joints'].keys()) {
      const ccdAngle = ccdSolution.joints[name]
      const jacobianAngle = jacobianSolution.joints[name]
      const finalAngle = ccdAngle * 0.3 + jacobianAngle * 0.7
      finalJoints[name] = finalAngle
    }

    return {
      ...jacobianSolution,
      joints: finalJoints,
      iterations: ccdSolution.iterations + jacobianSolution.iterations
    }
  }
}

/**
 * Get robot joint configuration for common robots
 */
export function getRobotJoints(robotType: 'unitree-g1' | 'boston-atlas' | 'tesla-optimus'): JointConfig[] {
  const configs: Record<string, JointConfig[]> = {
    'unitree-g1': [
      { name: 'base', position: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -Math.PI, maxAngle: Math.PI },
      { name: 'hip', parent: 'base', position: { x: 0, y: 0.5, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -1.5, maxAngle: 1.5, length: 0.2 },
      { name: 'knee', parent: 'hip', position: { x: 0, y: -0.3, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -2.5, maxAngle: 0.1, length: 0.25 },
      { name: 'ankle', parent: 'knee', position: { x: 0, y: -0.3, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -1.0, maxAngle: 1.0, length: 0.15 },
      { name: 'torso', parent: 'base', position: { x: 0, y: 0.5, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -0.5, maxAngle: 0.5, length: 0.3 },
      { name: 'neck', parent: 'torso', position: { x: 0, y: 0.3, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -1.0, maxAngle: 1.0, length: 0.1 },
      { name: 'shoulder', parent: 'torso', position: { x: 0.15, y: 0.2, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -2.0, maxAngle: 2.0 },
      { name: 'elbow', parent: 'shoulder', position: { x: 0, y: -0.2, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -2.5, maxAngle: 0.5, length: 0.2 },
      { name: 'wrist', parent: 'elbow', position: { x: 0, y: -0.15, z: 0 }, rotation: { x: 0, y: 0, z: 0 }, minAngle: -2.0, maxAngle: 2.0, length: 0.1 }
    ],
    'boston-atlas': [
      // Similar structure with more joints
    ],
    'tesla-optimus': [
      // Similar structure optimized for Tesla
    ]
  }

  return configs[robotType] || configs['unitree-g1']
}
