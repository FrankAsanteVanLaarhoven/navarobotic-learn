/**
 * WebSocket Server for Real-time Features
 * Handles live chat, video sessions, and real-time updates
 */

import { Server as WebSocketServer } from 'ws'
import { Server as HTTPServer } from 'http'

export interface WebSocketMessage {
  type: string
  payload: any
  userId?: string
  timestamp?: number
}

export class WebSocketService {
  private wss: WebSocketServer | null = null
  private clients: Map<string, Set<any>> = new Map() // userId -> Set of WebSocket connections

  /**
   * Initialize WebSocket server
   */
  initialize(server: HTTPServer): void {
    this.wss = new WebSocketServer({ server })

    this.wss.on('connection', (ws: any, req) => {
      // Extract user ID from query or headers
      const url = new URL(req.url || '', `http://${req.headers.host}`)
      const userId = url.searchParams.get('userId') || 'anonymous'

      // Add to clients map
      if (!this.clients.has(userId)) {
        this.clients.set(userId, new Set())
      }
      this.clients.get(userId)!.add(ws)

      console.log(`WebSocket client connected: ${userId}`)

      // Handle messages
      ws.on('message', (data: Buffer) => {
        try {
          const message: WebSocketMessage = JSON.parse(data.toString())
          this.handleMessage(ws, message, userId)
        } catch (error) {
          console.error('Error parsing WebSocket message:', error)
        }
      })

      // Handle disconnect
      ws.on('close', () => {
        const userClients = this.clients.get(userId)
        if (userClients) {
          userClients.delete(ws)
          if (userClients.size === 0) {
            this.clients.delete(userId)
          }
        }
        console.log(`WebSocket client disconnected: ${userId}`)
      })

      // Send welcome message
      this.sendToClient(ws, {
        type: 'connected',
        payload: { userId, timestamp: Date.now() }
      })
    })
  }

  /**
   * Handle incoming message
   */
  private handleMessage(ws: any, message: WebSocketMessage, userId: string): void {
    switch (message.type) {
      case 'chat':
        this.broadcastToRoom(message.payload.roomId, {
          type: 'chat',
          payload: {
            ...message.payload,
            userId,
            timestamp: Date.now()
          }
        })
        break

      case 'video_session':
        this.broadcastToRoom(message.payload.sessionId, {
          type: 'video_session',
          payload: {
            ...message.payload,
            userId,
            timestamp: Date.now()
          }
        })
        break

      case 'telemetry':
        // Broadcast robot telemetry updates
        this.broadcastToRoom(message.payload.sessionId, {
          type: 'telemetry',
          payload: {
            ...message.payload,
            userId,
            timestamp: Date.now()
          }
        })
        break

      case 'ping':
        this.sendToClient(ws, {
          type: 'pong',
          payload: { timestamp: Date.now() }
        })
        break

      default:
        console.warn(`Unknown message type: ${message.type}`)
    }
  }

  /**
   * Send message to specific client
   */
  private sendToClient(ws: any, message: WebSocketMessage): void {
    if (ws.readyState === 1) { // WebSocket.OPEN
      ws.send(JSON.stringify(message))
    }
  }

  /**
   * Broadcast message to all clients in a room
   */
  broadcastToRoom(roomId: string, message: WebSocketMessage): void {
    // For now, broadcast to all connected clients
    // In production, implement room-based routing
    this.clients.forEach((clients) => {
      clients.forEach((ws) => {
        this.sendToClient(ws, message)
      })
    })
  }

  /**
   * Send message to specific user
   */
  sendToUser(userId: string, message: WebSocketMessage): void {
    const userClients = this.clients.get(userId)
    if (userClients) {
      userClients.forEach((ws) => {
        this.sendToClient(ws, message)
      })
    }
  }

  /**
   * Broadcast to all connected clients
   */
  broadcast(message: WebSocketMessage): void {
    this.clients.forEach((clients) => {
      clients.forEach((ws) => {
        this.sendToClient(ws, message)
      })
    })
  }

  /**
   * Get connected clients count
   */
  getConnectedCount(): number {
    let count = 0
    this.clients.forEach((clients) => {
      count += clients.size
    })
    return count
  }
}

// Singleton instance
let wsService: WebSocketService | null = null

export function getWebSocketService(): WebSocketService {
  if (!wsService) {
    wsService = new WebSocketService()
  }
  return wsService
}
