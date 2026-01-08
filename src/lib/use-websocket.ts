/**
 * Client-side WebSocket hook for real-time collaboration
 */

'use client'

import { useEffect, useRef, useState, useCallback } from 'react'

export interface WebSocketMessage {
  type: string
  data?: any
  timestamp: number
}

export interface WebSocketState {
  isConnected: boolean
  userId: string
  userRole: 'student' | 'instructor' | 'admin'
  roomId: string
  users: number
}

export function useWebSocket(url: string) {
  const [socket, setSocket] = useState<WebSocket | null>(null)
  const [state, setState] = useState<WebSocketState>({
    isConnected: false,
    userId: '',
    userRole: 'student',
    roomId: '',
    users: 0
  })
  const reconnectTimeout = useRef<NodeJS.Timeout | null>(null)
  const messageHandlers = useRef<Map<string, (data: any) => void>>(new Map())

  const connect = useCallback(() => {
    try {
      const ws = new WebSocket(url)

      ws.onopen = () => {
        console.log('WebSocket connected')
        setSocket(ws)
        setState(prev => ({ ...prev, isConnected: true }))
      }

      ws.onmessage = (event: MessageEvent) => {
        try {
          const message: WebSocketMessage = JSON.parse(event.data)
          handleIncomingMessage(message)
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error)
        }
      }

      ws.onclose = () => {
        console.log('WebSocket disconnected')
        setSocket(null)
        setState(prev => ({ ...prev, isConnected: false }))
        
        reconnectTimeout.current = setTimeout(() => {
          connect()
        }, 5000)
      }

      ws.onerror = (error: Event) => {
        console.error('WebSocket error:', error)
        setSocket(null)
        setState(prev => ({ ...prev, isConnected: false }))
      }
    } catch (error) {
      console.error('Failed to connect to WebSocket:', error)
    }
  }, [url])

  const disconnect = useCallback(() => {
    if (socket) {
      socket.close()
    }
    if (reconnectTimeout.current) {
      clearTimeout(reconnectTimeout.current)
    }
  }, [socket])

  const sendMessage = useCallback((type: string, data: any) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      const message = {
        type,
        ...data,
        timestamp: Date.now()
      }
      socket.send(JSON.stringify(message))
    }
  }, [socket])

  const handleIncomingMessage = useCallback((message: WebSocketMessage) => {
    const handler = messageHandlers.current.get(message.type)
    if (handler) {
      handler(message.data || message)
    }

    switch (message.type) {
      case 'connection':
        setState(prev => ({ 
          ...prev, 
          userId: message.data?.userId || '', 
          userRole: message.data?.userRole || 'student' 
        }))
        break

      case 'activity':
        if (message.data?.action === 'join') {
          setState(prev => ({ ...prev, users: prev.users + 1 }))
        } else if (message.data?.action === 'leave') {
          setState(prev => ({ ...prev, users: Math.max(0, prev.users - 1) }))
        }
        break

      case 'joint_update':
      case 'gesture':
      case 'message':
      case 'course_progress':
      case 'instructor_control':
        // Handled by registered handlers
        break
    }
  }, [])

  const on = useCallback((event: string, handler: (data: any) => void) => {
    messageHandlers.current.set(event, handler)
  }, [])

  const off = useCallback((event: string) => {
    messageHandlers.current.delete(event)
  }, [])

  useEffect(() => {
    connect()
    return () => {
      disconnect()
    }
  }, [url])

  return {
    socket,
    state,
    connect,
    disconnect,
    sendMessage,
    on,
    off,
    isConnected: state.isConnected,
    users: state.users,
    roomId: state.roomId
  }
}
