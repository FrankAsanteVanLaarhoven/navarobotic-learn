/**
 * NeoVerse 4D World Model Integration
 * Enhances 3D simulations with 4D spatial-temporal modeling for realistic robot environments
 */

import * as THREE from 'three'

export interface NeoVerseConfig {
  enabled: boolean
  temporalConsistency: boolean
  spatialModeling: boolean
  realWorldPhysics: boolean
  cameraTracking: boolean
}

// Default config
const defaultConfig: NeoVerseConfig = {
  enabled: true,
  temporalConsistency: true,
  spatialModeling: true,
  realWorldPhysics: true,
  cameraTracking: true
}

// Helper to update config (for external use)
let globalConfig: NeoVerseConfig = { ...defaultConfig }

export function updateConfig(config: Partial<NeoVerseConfig>) {
  globalConfig = { ...globalConfig, ...config }
}

export interface NeoVerseWorldState {
  timestamp: number
  robotPosition: THREE.Vector3
  robotRotation: THREE.Euler
  environmentState: {
    lighting: THREE.Light[]
    objects: THREE.Object3D[]
    physics: any
  }
  cameraState: {
    position: THREE.Vector3
    target: THREE.Vector3
    mode: 'auto' | 'manual' | 'tracking'
  }
}

/**
 * NeoVerse 4D World Model Integration
 * Provides realistic spatial-temporal modeling for robot simulations
 */
export class NeoVerseIntegration {
  private config: NeoVerseConfig
  private worldHistory: NeoVerseWorldState[] = []
  private maxHistorySize = 60 // Keep 60 frames of history (2 seconds at 30fps)

  constructor(config: Partial<NeoVerseConfig> = {}) {
    this.config = {
      enabled: config.enabled ?? globalConfig.enabled ?? defaultConfig.enabled,
      temporalConsistency: config.temporalConsistency ?? globalConfig.temporalConsistency ?? defaultConfig.temporalConsistency,
      spatialModeling: config.spatialModeling ?? globalConfig.spatialModeling ?? defaultConfig.spatialModeling,
      realWorldPhysics: config.realWorldPhysics ?? globalConfig.realWorldPhysics ?? defaultConfig.realWorldPhysics,
      cameraTracking: config.cameraTracking ?? globalConfig.cameraTracking ?? defaultConfig.cameraTracking
    }
  }

  /**
   * Update world state with temporal consistency
   */
  updateWorldState(
    robotPosition: THREE.Vector3,
    robotRotation: THREE.Euler,
    scene: THREE.Scene,
    camera: THREE.Camera
  ): NeoVerseWorldState {
    const worldState: NeoVerseWorldState = {
      timestamp: Date.now(),
      robotPosition: robotPosition.clone(),
      robotRotation: robotRotation.clone(),
      environmentState: {
        lighting: scene.children.filter(child => child instanceof THREE.Light) as THREE.Light[],
        objects: scene.children.filter(child => 
          !(child instanceof THREE.Light) && 
          !(child instanceof THREE.Camera)
        ),
        physics: this.extractPhysicsState(scene)
      },
      cameraState: {
        position: camera.position.clone(),
        target: this.getCameraTarget(camera),
        mode: this.config.cameraTracking ? 'tracking' : 'auto'
      }
    }

    // Maintain temporal history
    if (this.config.temporalConsistency) {
      this.worldHistory.push(worldState)
      if (this.worldHistory.length > this.maxHistorySize) {
        this.worldHistory.shift()
      }
    }

    return worldState
  }

  /**
   * Get smoothed camera position based on temporal history
   */
  getSmoothCameraPosition(targetPosition: THREE.Vector3, smoothingFactor: number = 0.1): THREE.Vector3 {
    if (!this.config.temporalConsistency || this.worldHistory.length === 0) {
      return targetPosition
    }

    const lastState = this.worldHistory[this.worldHistory.length - 1]
    const lastCameraPos = lastState.cameraState.position

    // Smooth interpolation based on temporal consistency
    return lastCameraPos.clone().lerp(targetPosition, smoothingFactor)
  }

  /**
   * Get predicted robot position based on temporal history
   */
  getPredictedPosition(framesAhead: number = 1): THREE.Vector3 | null {
    if (!this.config.temporalConsistency || this.worldHistory.length < 2) {
      return null
    }

    const recent = this.worldHistory.slice(-5) // Use last 5 frames
    if (recent.length < 2) return null

    // Simple linear prediction
    const velocity = new THREE.Vector3()
      .subVectors(
        recent[recent.length - 1].robotPosition,
        recent[0].robotPosition
      )
      .divideScalar(recent.length - 1)

    const lastPos = recent[recent.length - 1].robotPosition
    return lastPos.clone().add(velocity.multiplyScalar(framesAhead))
  }

  /**
   * Enhance lighting with realistic global illumination
   */
  enhanceLighting(scene: THREE.Scene): void {
    if (!this.config.enabled || !this.config.spatialModeling) return

    // Add realistic ambient lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4)
    scene.add(ambientLight)

    // Add directional light with realistic shadows
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8)
    directionalLight.position.set(5, 10, 5)
    directionalLight.castShadow = true
    directionalLight.shadow.mapSize.width = 2048
    directionalLight.shadow.mapSize.height = 2048
    directionalLight.shadow.camera.near = 0.5
    directionalLight.shadow.camera.far = 50
    scene.add(directionalLight)

    // Add fill light for realistic lighting
    const fillLight = new THREE.DirectionalLight(0xffffff, 0.3)
    fillLight.position.set(-5, 5, -5)
    scene.add(fillLight)
  }

  /**
   * Apply real-world physics to objects
   */
  applyRealWorldPhysics(object: THREE.Object3D, deltaTime: number): void {
    if (!this.config.enabled || !this.config.realWorldPhysics) return

    // Apply gravity if object has physics properties
    if ((object as any).userData?.hasPhysics) {
      const velocity = (object as any).userData.velocity || new THREE.Vector3()
      const gravity = new THREE.Vector3(0, -9.81, 0) // Earth gravity
      
      velocity.add(gravity.multiplyScalar(deltaTime))
      object.position.add(velocity.multiplyScalar(deltaTime))
      
      (object as any).userData.velocity = velocity
    }
  }

  /**
   * Get optimal camera position for tracking robot
   */
  getOptimalCameraPosition(
    robotPosition: THREE.Vector3,
    robotRotation: THREE.Euler,
    preferredDistance: number = 5
  ): { position: THREE.Vector3; target: THREE.Vector3 } {
    if (!this.config.cameraTracking) {
      return {
        position: new THREE.Vector3(0, 2, preferredDistance),
        target: robotPosition.clone()
      }
    }

    // Calculate camera position based on robot's facing direction
    const forward = new THREE.Vector3(0, 0, -1)
    forward.applyEuler(robotRotation)
    
    const cameraOffset = forward.multiplyScalar(-preferredDistance)
    cameraOffset.y = 2 // Height offset

    const cameraPosition = robotPosition.clone().add(cameraOffset)
    const cameraTarget = robotPosition.clone()
    cameraTarget.y += 1.5 // Look at robot's upper body

    return {
      position: cameraPosition,
      target: cameraTarget
    }
  }

  /**
   * Extract physics state from scene
   */
  private extractPhysicsState(scene: THREE.Scene): any {
    // Extract relevant physics information
    return {
      gravity: -9.81,
      objects: scene.children
        .filter(obj => (obj as any).userData?.hasPhysics)
        .map(obj => ({
          position: obj.position.toArray(),
          rotation: obj.rotation.toArray(),
          velocity: (obj as any).userData?.velocity?.toArray() || [0, 0, 0]
        }))
    }
  }

  /**
   * Get camera target from camera
   */
  private getCameraTarget(camera: THREE.Camera): THREE.Vector3 {
    if (camera instanceof THREE.PerspectiveCamera || camera instanceof THREE.OrthographicCamera) {
      const direction = new THREE.Vector3(0, 0, -1)
      direction.applyQuaternion(camera.quaternion)
      return camera.position.clone().add(direction.multiplyScalar(10))
    }
    return new THREE.Vector3(0, 0, 0)
  }

  /**
   * Generate video prompt description for NeoVerse features
   */
  generateVideoPromptDescription(): string {
    if (!this.config.enabled) return ''

    return `
NeoVerse 4D World Model Features:
- Temporal consistency: Smooth motion across frames with 4D spatial-temporal modeling
- Spatial modeling: Realistic 3D environment with depth and spatial awareness
- Real-world physics: Accurate gravity, friction, and material interactions
- Camera tracking: Cinematic camera with 3D spatial awareness, smooth tracking
- Global illumination: Realistic lighting with shadows and reflections
- Material properties: Photorealistic textures and surface properties
`
  }

  /**
   * Reset world history
   */
  reset(): void {
    this.worldHistory = []
  }
}

// Export singleton instance
export const neoverseIntegration = new NeoVerseIntegration()
