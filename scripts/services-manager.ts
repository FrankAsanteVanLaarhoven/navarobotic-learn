#!/usr/bin/env bun

/**
 * Services Manager
 * Automatically starts and manages all required services for the platform
 * - ROSBridge WebSocket server
 * - Any other required background services
 */

import { spawn, ChildProcess } from 'child_process'
import { existsSync } from 'fs'
import { join } from 'path'

interface Service {
  name: string
  process: ChildProcess | null
  command: string
  args: string[]
  enabled: boolean
  healthCheck?: () => Promise<boolean>
}

class ServicesManager {
  private services: Map<string, Service> = new Map()
  private isShuttingDown = false

  constructor() {
    // Setup graceful shutdown
    process.on('SIGINT', () => this.shutdown())
    process.on('SIGTERM', () => this.shutdown())
    process.on('exit', () => this.shutdown())
  }

  /**
   * Register ROSBridge service
   */
  registerROSBridge(): void {
    // Check if ROS is available
    const rosVersion = this.checkROSInstallation()
    if (!rosVersion) {
      console.warn('⚠️  ROS not found. ROSBridge will not start.')
      console.warn('   Install ROS2: https://docs.ros.org/en/humble/Installation.html')
      return
    }

    const rosDistro = rosVersion
    const rosBridgeCommand = `ros2`
    const rosBridgeArgs = [
      'launch',
      'rosbridge_server',
      'rosbridge_websocket.launch.py'
    ]

    // Check if rosbridge is installed
    if (!this.checkROSBridgeInstalled(rosDistro)) {
      console.warn(`⚠️  ROSBridge not installed for ${rosDistro}`)
      console.warn(`   Install with: sudo apt-get install ros-${rosDistro}-rosbridge-suite`)
      return
    }

    this.services.set('rosbridge', {
      name: 'ROSBridge',
      process: null,
      command: rosBridgeCommand,
      args: rosBridgeArgs,
      enabled: true,
      healthCheck: async () => {
        try {
          const response = await fetch('http://localhost:9090', { 
            method: 'GET',
            signal: AbortSignal.timeout(1000)
          })
          return response.status < 500
        } catch {
          return false
        }
      }
    })
  }

  /**
   * Check if ROS is installed
   */
  private checkROSInstallation(): string | null {
    // Try to detect ROS version
    const commonPaths = [
      '/opt/ros/humble',
      '/opt/ros/foxy',
      '/opt/ros/galactic',
      '/opt/homebrew/opt/ros/humble',
      '/usr/local/ros/humble'
    ]

    for (const path of commonPaths) {
      if (existsSync(path)) {
        // Extract distro from path
        const distro = path.split('/').pop()
        if (distro) return distro
      }
    }

    // Try to run rosversion command
    try {
      const { execSync } = require('child_process')
      const version = execSync('rosversion -d 2>/dev/null', { encoding: 'utf-8', timeout: 2000 }).trim()
      if (version) return version
    } catch {
      // ROS not found
    }

    return null
  }

  /**
   * Check if ROSBridge is installed
   */
  private checkROSBridgeInstalled(distro: string): boolean {
    try {
      const { execSync } = require('child_process')
      execSync(`ros2 pkg list | grep -q rosbridge`, { encoding: 'utf-8', timeout: 2000 })
      return true
    } catch {
      return false
    }
  }

  /**
   * Start a service
   */
  async startService(name: string): Promise<boolean> {
    const service = this.services.get(name)
    if (!service || !service.enabled) {
      return false
    }

    if (service.process && !service.process.killed) {
      console.log(`✅ ${service.name} is already running`)
      return true
    }

    console.log(`🚀 Starting ${service.name}...`)

    try {
      // Source ROS environment if needed
      const env = { ...process.env }
      const rosDistro = this.checkROSInstallation()
      
      if (rosDistro && name === 'rosbridge') {
        // Try to source ROS setup
        const rosPath = `/opt/ros/${rosDistro}`
        if (existsSync(rosPath)) {
          env.ROS_DISTRO = rosDistro
          // Note: In production, you'd want to source the setup.bash
          // For now, we rely on system PATH
        }
      }

      const process = spawn(service.command, service.args, {
        env,
        stdio: 'pipe',
        shell: false
      })

      process.stdout?.on('data', (data) => {
        const output = data.toString().trim()
        if (output) {
          console.log(`[${service.name}] ${output}`)
        }
      })

      process.stderr?.on('data', (data) => {
        const output = data.toString().trim()
        if (output && !output.includes('INFO')) {
          console.error(`[${service.name}] ${output}`)
        }
      })

      process.on('exit', (code) => {
        if (!this.isShuttingDown) {
          console.error(`❌ ${service.name} exited with code ${code}`)
          // Auto-restart after 3 seconds
          setTimeout(() => {
            if (!this.isShuttingDown) {
              console.log(`🔄 Restarting ${service.name}...`)
              this.startService(name)
            }
          }, 3000)
        }
      })

      service.process = process

      // Wait a bit and check health
      await new Promise(resolve => setTimeout(resolve, 2000))
      
      if (service.healthCheck) {
        const isHealthy = await service.healthCheck()
        if (isHealthy) {
          console.log(`✅ ${service.name} started successfully`)
          return true
        } else {
          console.warn(`⚠️  ${service.name} started but health check failed`)
          return true // Still return true, service might be starting
        }
      } else {
        console.log(`✅ ${service.name} started`)
        return true
      }
    } catch (error: any) {
      console.error(`❌ Failed to start ${service.name}:`, error.message)
      return false
    }
  }

  /**
   * Stop a service
   */
  stopService(name: string): void {
    const service = this.services.get(name)
    if (!service || !service.process) {
      return
    }

    console.log(`🛑 Stopping ${service.name}...`)
    service.process.kill('SIGTERM')
    service.process = null
  }

  /**
   * Start all enabled services
   */
  async startAll(): Promise<void> {
    console.log('🚀 Starting all services...\n')
    
    // Register all services
    this.registerROSBridge()

    // Start all enabled services
    const startPromises = Array.from(this.services.values())
      .filter(s => s.enabled)
      .map(s => this.startService(s.name))

    await Promise.all(startPromises)
    
    console.log('\n✅ All services started')
    console.log('   Press Ctrl+C to stop all services\n')
  }

  /**
   * Stop all services
   */
  shutdown(): void {
    if (this.isShuttingDown) return
    this.isShuttingDown = true

    console.log('\n🛑 Shutting down all services...')
    
    for (const [name] of this.services) {
      this.stopService(name)
    }

    // Give processes time to shutdown gracefully
    setTimeout(() => {
      process.exit(0)
    }, 2000)
  }

  /**
   * Get status of all services
   */
  getStatus(): Record<string, { running: boolean; enabled: boolean }> {
    const status: Record<string, { running: boolean; enabled: boolean }> = {}
    
    for (const [name, service] of this.services) {
      status[name] = {
        running: service.process !== null && !service.process.killed,
        enabled: service.enabled
      }
    }

    return status
  }

  /**
   * Get all enabled service names
   */
  getEnabledServiceNames(): string[] {
    return Array.from(this.services.values())
      .filter(s => s.enabled)
      .map(s => s.name)
  }
}

// Main execution
if (import.meta.main) {
  const manager = new ServicesManager()
  
  // Start all services
  manager.startAll().catch((error) => {
    console.error('Failed to start services:', error)
    process.exit(1)
  })

  // Keep process alive
  process.stdin.resume()
}

export { ServicesManager }
