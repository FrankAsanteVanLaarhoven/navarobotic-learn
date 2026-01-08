/**
 * Code Execution Service
 * Supports Docker containers and AWS Lambda
 */

import { exec } from 'child_process'
import { promisify } from 'util'
import Docker from 'dockerode'

const execAsync = promisify(exec)

export interface CodeExecutionConfig {
  provider: 'docker' | 'lambda'
  timeout?: number
  memoryLimit?: string
}

export interface CodeExecutionRequest {
  code: string
  language: 'python' | 'javascript' | 'cpp' | 'rust'
  input?: string
  timeout?: number
}

export interface CodeExecutionResult {
  success: boolean
  output?: string
  error?: string
  executionTime?: number
  memoryUsed?: number
}

export class CodeExecutionService {
  private provider: 'docker' | 'lambda'
  private docker: Docker | null = null
  private timeout: number
  private memoryLimit: string

  constructor(config: CodeExecutionConfig) {
    this.provider = config.provider
    this.timeout = config.timeout || 30000 // 30 seconds default
    this.memoryLimit = config.memoryLimit || '512m'

    if (config.provider === 'docker') {
      try {
        this.docker = new Docker()
      } catch (error) {
        console.warn('Docker not available, code execution will be limited')
      }
    }
  }

  /**
   * Execute code in a sandboxed environment
   */
  async executeCode(request: CodeExecutionRequest): Promise<CodeExecutionResult> {
    if (this.provider === 'docker') {
      return await this.executeWithDocker(request)
    } else {
      return await this.executeWithLambda(request)
    }
  }

  /**
   * Execute code using Docker container
   */
  private async executeWithDocker(
    request: CodeExecutionRequest
  ): Promise<CodeExecutionResult> {
    if (!this.docker) {
      return {
        success: false,
        error: 'Docker not available'
      }
    }

    const startTime = Date.now()
    const image = this.getDockerImage(request.language)

    try {
      // Create container
      const container = await this.docker.createContainer({
        Image: image,
        Cmd: this.getExecutionCommand(request.language, request.code),
        AttachStdout: true,
        AttachStderr: true,
        Memory: this.memoryLimitToBytes(this.memoryLimit),
        NetworkDisabled: true, // Security: disable network access
        HostConfig: {
          Memory: this.memoryLimitToBytes(this.memoryLimit),
          CpuQuota: 50000, // Limit CPU usage
          PidsLimit: 10 // Limit process count
        }
      })

      // Start container
      await container.start()

      // Wait for execution with timeout
      const timeoutPromise = new Promise<void>((_, reject) => {
        setTimeout(() => reject(new Error('Execution timeout')), request.timeout || this.timeout)
      })

      const executionPromise = container.wait()

      try {
        await Promise.race([executionPromise, timeoutPromise])
      } catch (error: any) {
        if (error.message === 'Execution timeout') {
          await container.stop()
          await container.remove()
          return {
            success: false,
            error: 'Execution timeout',
            executionTime: Date.now() - startTime
          }
        }
        throw error
      }

      // Get logs
      const logs = await container.logs({
        stdout: true,
        stderr: true
      })

      const output = logs.toString('utf-8')

      // Clean up
      await container.remove()

      const executionTime = Date.now() - startTime

      return {
        success: true,
        output,
        executionTime
      }
    } catch (error: any) {
      return {
        success: false,
        error: error.message || 'Execution failed',
        executionTime: Date.now() - startTime
      }
    }
  }

  /**
   * Execute code using AWS Lambda (placeholder)
   */
  private async executeWithLambda(
    request: CodeExecutionRequest
  ): Promise<CodeExecutionResult> {
    // TODO: Implement AWS Lambda execution
    // This would require AWS SDK and Lambda function setup
    return {
      success: false,
      error: 'Lambda execution not yet implemented'
    }
  }

  /**
   * Get Docker image for language
   */
  private getDockerImage(language: string): string {
    const images: Record<string, string> = {
      python: 'python:3.11-slim',
      javascript: 'node:20-slim',
      cpp: 'gcc:latest',
      rust: 'rust:1.75-slim'
    }
    return images[language] || 'python:3.11-slim'
  }

  /**
   * Get execution command for language
   */
  private getExecutionCommand(language: string, code: string): string[] {
    switch (language) {
      case 'python':
        return ['python', '-c', code]
      case 'javascript':
        return ['node', '-e', code]
      case 'cpp':
        // For C++, we'd need to compile first
        return ['sh', '-c', `echo "${code}" > /tmp/code.cpp && g++ /tmp/code.cpp -o /tmp/code && /tmp/code`]
      case 'rust':
        // For Rust, we'd need to compile first
        return ['sh', '-c', `echo "${code}" > /tmp/code.rs && rustc /tmp/code.rs -o /tmp/code && /tmp/code`]
      default:
        return ['echo', 'Unsupported language']
    }
  }

  /**
   * Convert memory limit string to bytes
   */
  private memoryLimitToBytes(limit: string): number {
    const units: Record<string, number> = {
      b: 1,
      kb: 1024,
      mb: 1024 * 1024,
      gb: 1024 * 1024 * 1024
    }

    const match = limit.toLowerCase().match(/^(\d+)([a-z]+)$/)
    if (!match) return 512 * 1024 * 1024 // Default 512MB

    const value = parseInt(match[1])
    const unit = match[2]
    return value * (units[unit] || units.mb)
  }
}

// Singleton instance
let codeExecutionService: CodeExecutionService | null = null

export function getCodeExecutionService(): CodeExecutionService | null {
  if (codeExecutionService) return codeExecutionService

  const provider = (process.env.CODE_EXECUTION_PROVIDER || 'docker') as 'docker' | 'lambda'
  const timeout = parseInt(process.env.CODE_EXECUTION_TIMEOUT || '30000')
  const memoryLimit = process.env.CODE_EXECUTION_MEMORY_LIMIT || '512m'

  codeExecutionService = new CodeExecutionService({
    provider,
    timeout,
    memoryLimit
  })

  return codeExecutionService
}
