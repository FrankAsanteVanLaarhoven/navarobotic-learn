/**
 * WebROS Cloud IDE Core
 * Provides a browser-based robotics development environment.
 * Integrates code editing, file management, and direct robot control.
 */

export interface EditorState {
  code: string
  language: string
  filename: string
  cursor: { line: number; ch: number }
  isDirty: boolean
}

export interface TerminalOutput {
  id: string
  timestamp: number
  type: 'stdout' | 'stderr' | 'info' | 'error' | 'warning'
  source: string // 'ros_master', 'robot', 'system'
  content: string
}

export interface ROSNodeStatus {
  name: string
  status: 'running' | 'stopped' | 'error'
  pid?: number
  memory?: number
  cpu?: number
}

export class WebROSIDE {
  private rosConnection: WebSocket | null = null // WebSocket connection for ROS bridge
  private wsUrl: string = ''
  private editorState: EditorState = {
    code: '#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\n\ndef main():\n    rclpy.init()\n    node = Node("minimal_robot")\n    print("Robot Node Started")\n    rclpy.spin()\n    rclpy.shutdown()\n\nif __name__ == \'__main__\':\n    main()\n',
    language: 'python',
    filename: 'main.py',
    cursor: { line: 1, ch: 0 },
    isDirty: false
  }
  private terminalHistory: TerminalOutput[] = []
  private fileSystem: Map<string, { content: string; language: string }> = new Map()
  private runningNodes: Map<string, ROSNodeStatus> = new Map()

  /**
   * Initialize the IDE and connect to ROS via WebSocket
   */
  async initialize(wsUrl?: string): Promise<void> {
    // Get ROS Bridge URL from environment or use default
    this.wsUrl = wsUrl || process.env.NEXT_PUBLIC_ROS_BRIDGE_URL || 'ws://localhost:9090'
    
    try {
      // Connect to ROSBridge WebSocket
      this.rosConnection = new WebSocket(this.wsUrl)
      
      this.rosConnection.onopen = () => {
        this.logToTerminal('info', 'system', 'Connected to ROS Bridge')
        
        // Subscribe to ROS topics
        this.sendROSMessage({
          op: 'subscribe',
          topic: '/rosout',
          type: 'rosgraph_msgs/Log'
        })
        
        this.sendROSMessage({
          op: 'subscribe',
          topic: '/robot_state',
          type: 'std_msgs/String'
        })
      }

      this.rosConnection.onmessage = (event: MessageEvent) => {
        try {
          const data = JSON.parse(event.data)
          this.handleROSMessage(data)
        } catch (error) {
          console.error('Failed to parse ROS message:', error)
        }
      }

      this.rosConnection.onerror = (error: Event) => {
        this.logToTerminal('error', 'system', 'ROS Bridge connection error')
        console.error('ROS WebSocket error:', error)
      }

      this.rosConnection.onclose = () => {
        this.logToTerminal('warning', 'system', 'ROS Bridge connection closed')
        this.rosConnection = null
      }

      // Setup default file system structure
      this.fileSystem.set('main.py', {
        content: this.editorState.code,
        language: 'python'
      })
      this.fileSystem.set('launch_file.launch', {
        content: '<launch>\n  <node pkg="unitree_g1_ros" exec="robot_state_publisher" name="robot_state_publisher" />\n  <include file="$(find unitree_g1_description)/launch/robot_control.launch" />\n</launch>',
        language: 'xml'
      })
      this.fileSystem.set('CMakeLists.txt', {
        content: 'cmake_minimum_required(VERSION 3.0.2)\nproject(unitree_g1_ros)\nfind_package(catkin REQUIRED COMPONENTS roscpp)\nfind_package(rospy REQUIRED COMPONENTS rospy)\ncatkin_package(...)',
        language: 'cmake'
      })

      this.logToTerminal('info', 'system', 'WebROS IDE Initialized')
    } catch (error) {
      this.logToTerminal('error', 'system', 'Failed to initialize WebROS IDE. Using offline mode.')
      console.error('WebROS initialization error:', error)
    }
  }

  /**
   * Send message to ROS Bridge
   */
  private sendROSMessage(message: any): void {
    if (this.rosConnection && this.rosConnection.readyState === WebSocket.OPEN) {
      this.rosConnection.send(JSON.stringify(message))
    }
  }

  /**
   * Handle incoming ROS messages
   */
  private handleROSMessage(data: any): void {
    if (data.op === 'publish' && data.topic === '/rosout') {
      const level = data.msg?.level || 2 // 2 = INFO
      const msg = data.msg?.msg || ''
      
      if (level >= 8) { // ERROR
        this.logToTerminal('error', 'ros_master', msg)
      } else if (level >= 4) { // WARN
        this.logToTerminal('warning', 'ros_master', msg)
      } else {
        this.logToTerminal('stdout', 'ros_master', msg)
      }
    } else if (data.op === 'publish' && data.topic === '/robot_state') {
      this.logToTerminal('info', 'robot', data.msg?.data || 'Robot state update')
    }
  }

  /**
   * Update code in the editor
   */
  updateCode(newCode: string): void {
    this.editorState.code = newCode
    this.editorState.isDirty = true
    
    // Auto-save to file system
    if (this.fileSystem.has(this.editorState.filename)) {
      this.fileSystem.set(this.editorState.filename, {
        ...this.fileSystem.get(this.editorState.filename)!,
        content: newCode
      })
    }
  }

  /**
   * Deploy code to the robot (Build & Run)
   */
  async deployToRobot(): Promise<void> {
    this.logToTerminal('info', 'system', 'Starting deployment...')

    try {
      // 1. Validate Syntax (Basic check)
      if (this.editorState.language === 'python') {
        const isValid = this.validatePythonSyntax(this.editorState.code)
        if (!isValid) {
          this.logToTerminal('error', 'system', 'Syntax Error: Check your Python code.')
          return
        }
      }

      // 2. Send code to robot via ROS Bridge
      // Publish to a ROS service/topic for code execution
      this.sendROSMessage({
        op: 'publish',
        topic: '/webros/execute_code',
        msg: {
          filename: this.editorState.filename,
          language: this.editorState.language,
          code: this.editorState.code
        }
      })

      // Also call a ROS service if available
      this.sendROSMessage({
        op: 'call_service',
        service: '/webros/execute_script',
        args: {
          filename: this.editorState.filename,
          language: this.editorState.language,
          code: this.editorState.code
        }
      })

      // 3. Mark as clean
      this.editorState.isDirty = false
      this.logToTerminal('info', 'robot', `Deployed ${this.editorState.filename} successfully.`)

    } catch (error: any) {
      this.logToTerminal('error', 'system', `Deployment failed: ${error?.message || error}`)
    }
  }

  /**
   * Stop running code/node
   */
  async stopExecution(): Promise<void> {
    this.sendROSMessage({
      op: 'publish',
      topic: '/webros/stop_execution',
      msg: { command: 'stop' }
    })
    this.logToTerminal('info', 'robot', 'Execution stopped.')
  }

  /**
   * Manage file system (Open, Save, Create, Delete)
   */
  openFile(filename: string): void {
    if (!this.fileSystem.has(filename)) return

    const file = this.fileSystem.get(filename)!
    this.editorState.code = file.content
    this.editorState.language = file.language
    this.editorState.filename = filename
    this.editorState.isDirty = false
    
    this.logToTerminal('info', 'system', `Opened ${filename}`)
  }

  saveFile(): void {
    this.logToTerminal('info', 'system', `Saved ${this.editorState.filename}`)
    this.editorState.isDirty = false
  }

  createFile(filename: string, language: string = 'python'): void {
    const extension = language === 'python' ? '.py' : language === 'xml' ? '.launch' : '.txt'
    const defaultContent = language === 'python' 
      ? '#!/usr/bin/env python3\n# New Script\n' 
      : language === 'xml' 
      ? '<launch>\n  <!-- New Launch File -->\n</launch>' 
      : ''

    this.fileSystem.set(filename + extension, {
      content: defaultContent,
      language
    })
    this.openFile(filename + extension)
  }

  deleteFile(filename: string): void {
    this.fileSystem.delete(filename)
    this.logToTerminal('info', 'system', `Deleted ${filename}`)
  }

  /**
   * Terminal Management
   */
  logToTerminal(type: TerminalOutput['type'], source: string, message: string): void {
    const output: TerminalOutput = {
      id: `${Date.now()}-${Math.random()}`,
      timestamp: Date.now(),
      type,
      source,
      content: message
    }

    this.terminalHistory.push(output)
    
    // Limit history size
    if (this.terminalHistory.length > 1000) {
      this.terminalHistory = this.terminalHistory.slice(-500)
    }
  }

  clearTerminal(): void {
    this.terminalHistory = []
    this.logToTerminal('info', 'system', 'Terminal cleared.')
  }

  /**
   * Internal Helpers
   */
  private validatePythonSyntax(code: string): boolean {
    // Very basic check. A real IDE would use AST or Pylint.
    try {
      // Check for unmatched brackets/parentheses
      const openBrackets = (code.match(/\(/g) || []).length
      const closeBrackets = (code.match(/\)/g) || []).length
      if (openBrackets !== closeBrackets) return false

      return true
    } catch {
      return false
    }
  }

  getState(): {
    editor: EditorState
    files: string[]
    terminal: TerminalOutput[]
    nodes: ROSNodeStatus[]
  } {
    return {
      editor: this.editorState,
      files: Array.from(this.fileSystem.keys()),
      terminal: this.terminalHistory,
      nodes: Array.from(this.runningNodes.values())
    }
  }
}
