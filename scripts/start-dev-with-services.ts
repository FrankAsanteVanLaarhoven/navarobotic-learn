#!/usr/bin/env bun

/**
 * Development Server with Auto-Started Services
 * Starts Next.js dev server along with all required services (ROSBridge, etc.)
 */

import { spawn } from 'child_process'
import { ServicesManager } from './services-manager'

const manager = new ServicesManager()

// Start services in background
const startServices = async () => {
  // Register services first
  manager.registerROSBridge()
  
  // Get all enabled services and start them
  const serviceNames = ['rosbridge'] // Service names to start
  
  for (const serviceName of serviceNames) {
    try {
      await manager.startService(serviceName)
      // Small delay between service starts
      await new Promise(resolve => setTimeout(resolve, 1000))
    } catch (error) {
      console.warn(`⚠️  Failed to start ${serviceName}, continuing...`)
    }
  }
}

// Start services
startServices().then(() => {
  console.log('\n🚀 Starting Next.js development server...\n')
  
  // Start Next.js dev server
  const nextDev = spawn('bun', ['run', 'next', 'dev', '-p', '3001'], {
    stdio: 'inherit',
    shell: false
  })

  nextDev.on('exit', (code) => {
    console.log(`\n🛑 Next.js dev server exited with code ${code}`)
    manager.shutdown()
    process.exit(code || 0)
  })

  // Handle shutdown
  process.on('SIGINT', () => {
    console.log('\n🛑 Shutting down...')
    nextDev.kill('SIGTERM')
    manager.shutdown()
  })

  process.on('SIGTERM', () => {
    nextDev.kill('SIGTERM')
    manager.shutdown()
  })
}).catch((error) => {
  console.error('❌ Failed to start services:', error)
  // Still start Next.js even if services fail
  console.log('🚀 Starting Next.js development server anyway...\n')
  
  const nextDev = spawn('bun', ['run', 'next', 'dev', '-p', '3001'], {
    stdio: 'inherit',
    shell: false
  })

  nextDev.on('exit', (code) => {
    process.exit(code || 0)
  })
})
