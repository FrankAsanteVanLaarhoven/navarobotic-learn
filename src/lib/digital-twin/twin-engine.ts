/**
 * Digital Twin Engine
 * Creates a bi-directional sync between a simulation and physical hardware.
 * Visualizes the state in real-time.
 */

export interface TwinState {
  id: string
  name: string
  isPhysical: boolean
  position: { x: number; y: number; z: number }
  rotation: { x: number; y: number; z: number; w: number }
  velocity: { x: number; y: number; z: number }
  battery: number
  status: 'connected' | 'disconnected' | 'syncing' | 'error'
  lastSync: number
}

export class DigitalTwinEngine {
  private twins: Map<string, TwinState> = new Map()
  private syncInterval: number = 50 // ms (20 Hz)
  private autoSyncEnabled = true
  private isSyncing = false
  private subscribers: Map<string, (twin: TwinState) => void> = new Map()

  constructor() {
    this.startAutoSync()
  }

  /**
   * Register a physical robot
   */
  registerPhysicalRobot(id: string, name: string, connection: any): void {
    const twin: TwinState = {
      id,
      name,
      isPhysical: true,
      position: { x: 0, y: 0, z: 0 },
      rotation: { x: 0, y: 0, z: 0, w: 1 },
      velocity: { x: 0, y: 0, z: 0 },
      battery: 100,
      status: 'connected',
      lastSync: Date.now()
    }

    this.twins.set(id, twin)
    console.log(`Physical robot registered: ${name}`)

    // Listen to hardware updates
    if (connection && connection.on) {
      connection.on('telemetry', (data: any) => {
        this.updateTwinState(id, {
          position: data.position,
          velocity: data.velocity,
          battery: data.battery
        })
      })
    }
  }

  /**
   * Register a simulation twin
   */
  registerSimulationTwin(id: string, name: string, sceneObject: any): void {
    const twin: TwinState = {
      id,
      name,
      isPhysical: false,
      position: { x: sceneObject.position.x, y: sceneObject.position.y, z: sceneObject.position.z },
      rotation: { x: 0, y: 0, z: 0, w: 1 }, // Simplified
      velocity: { x: 0, y: 0, z: 0 },
      battery: 100,
      status: 'connected',
      lastSync: Date.now()
    }

    this.twins.set(id, twin)
    console.log(`Simulation twin registered: ${name}`)
  }

  /**
   * Update twin state (from simulation or hardware)
   */
  updateTwinState(twinId: string, state: Partial<TwinState>): void {
    const twin = this.twins.get(twinId)
    if (!twin) return

    // Merge state
    const updatedTwin = { ...twin, ...state, lastSync: Date.now() }
    this.twins.set(twinId, updatedTwin)

    // Notify subscribers
    this.notifySubscribers(updatedTwin)

    // Bi-directional sync: If this is the sim, push to physical.
    // If this is physical, update sim visualization.
    this.triggerSync(updatedTwin)
  }

  /**
   * Trigger sync event
   */
  private triggerSync(twin: TwinState): void {
    // In a real app, this emits an event
    // "SYNC_TWIN" with the new state
    // Subscribers (e.g., the 3D Scene) update their visuals
    // The physical robot receives commands to match the state
    console.log(`Sync triggered for ${twin.name} (Physical: ${twin.isPhysical})`)
  }

  /**
   * Subscribe to twin updates
   */
  subscribe(twinId: string, callback: (twin: TwinState) => void): void {
    this.subscribers.set(`${twinId}-${callback.name || 'anon'}`, callback)
  }

  /**
   * Unsubscribe
   */
  unsubscribe(twinId: string, callback: (twin: TwinState) => void): void {
    // Find and remove the specific subscriber
    for (const [key, cb] of this.subscribers.entries()) {
      if (key.startsWith(twinId) && cb === callback) {
        this.subscribers.delete(key)
        break
      }
    }
  }

  /**
   * Notify all subscribers of a specific twin
   */
  private notifySubscribers(twin: TwinState): void {
    for (const [key, callback] of this.subscribers.entries()) {
      if (key.startsWith(twin.id)) {
        try {
          callback(twin)
        } catch (e) {
          console.error('Subscriber error:', e)
        }
      }
    }
  }

  /**
   * Start auto-sync loop
   */
  private startAutoSync(): void {
    if (this.autoSyncEnabled && !this.isSyncing) {
      setInterval(() => {
        if (this.autoSyncEnabled) {
          this.performSync()
        }
      }, this.syncInterval)
    }
  }

  /**
   * Stop auto-sync
   */
  stopAutoSync(): void {
    this.autoSyncEnabled = false
  }

  /**
   * Perform bi-directional synchronization
   */
  private async performSync(): Promise<void> {
    if (this.isSyncing) return
    this.isSyncing = true

    // Logic: 
    // 1. Compare states of Sim vs Physical.
    // 2. If Physical moved (e.g., wind), update Sim.
    // 3. If Sim moved (e.g., user teleop), send command to Physical.
    
    // For now, we simulate this logic.
    for (const [id, twin] of this.twins) {
      // If physical is online, ensure visual twin matches it
      if (twin.isPhysical && twin.status === 'connected') {
        // Update visual twin to match physical
        // (In 3D scene component, this would update mesh position)
      }
    }

    this.isSyncing = false
  }

  /**
   * Inject noise/sensor emulation
   * Useful for training Sim2Real models
   */
  injectNoise(twinId: string, noiseLevel: number): void {
    const twin = this.twins.get(twinId)
    if (!twin) return

    // Add random noise to position (simulating sensor drift)
    const noise = () => (Math.random() - 0.5) * noiseLevel
    
    twin.position.x += noise()
    twin.position.y += noise()
    twin.position.z += noise()

    this.twins.set(twinId, twin)
  }

  /**
   * Get all twins
   */
  getAllTwins(): TwinState[] {
    return Array.from(this.twins.values())
  }

  /**
   * Get specific twin
   */
  getTwin(id: string): TwinState | undefined {
    return this.twins.get(id)
  }
}

/**
 * Factory function
 */
export function createDigitalTwinEngine(): DigitalTwinEngine {
  return new DigitalTwinEngine()
}
