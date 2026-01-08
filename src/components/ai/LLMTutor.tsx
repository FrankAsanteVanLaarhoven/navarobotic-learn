'use client'

import { useState, useRef, useEffect } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { motion, AnimatePresence } from 'framer-motion'
import {
  Send, Sparkles, Loader2, Code, BookOpen,
  Lightbulb, AlertCircle, CheckCircle, Copy,
  RefreshCw, MessageSquare, Bot, User
} from 'lucide-react'

interface Message {
  id: string
  role: 'user' | 'assistant' | 'system'
  content: string
  timestamp: number
  type?: 'text' | 'code' | 'explanation'
}

interface TutorState {
  isProcessing: boolean
  mode: 'socratic' | 'explanation' | 'coding'
  currentContext: string
  history: Message[]
}

export function LLMTutor() {
  const [input, setInput] = useState('')
  const [messages, setMessages] = useState<Message[]>([
    { id: '1', role: 'system', content: "Hi! I'm your AI Robotics Tutor. I can help you with ROS2, kinematics, physics, and code generation.", timestamp: Date.now(), type: 'explanation' }
  ])
  const [mode, setMode] = useState<'socratic' | 'explanation' | 'coding'>('explanation')
  const [isProcessing, setIsProcessing] = useState(false)
  const messagesEndRef = useRef<HTMLDivElement>(null)

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [messages])

  const handleSend = async () => {
    if (!input.trim()) return

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: Date.now(),
      type: 'text'
    }

    setMessages(prev => [...prev, userMessage])
    const currentInput = input
    setInput('')
    setIsProcessing(true)

    try {
      // Call the AI Tutor API
      const response = await fetch('/api/ai/tutor', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          prompt: currentInput,
          mode,
          history: messages.slice(-10) // Send last 10 messages for context
        })
      })

      if (!response.ok) {
        throw new Error('API request failed')
      }

      const data = await response.json()
      
      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.content || 'I apologize, but I could not generate a response.',
        timestamp: Date.now(),
        type: data.type || 'explanation'
      }

      setMessages(prev => [...prev, aiMessage])
    } catch (error) {
      console.error('LLM Tutor error:', error)
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'system',
        content: 'Sorry, I encountered an error processing your request. Please check your API configuration.',
        timestamp: Date.now(),
        type: 'explanation'
      }
      setMessages(prev => [...prev, errorMessage])
    } finally {
      setIsProcessing(false)
    }
  }

  return (
    <div className="h-full flex flex-col bg-[#0d0d1a]/50 backdrop-blur-sm border border-slate-700">
      {/* Header */}
      <CardHeader className="pb-3 border-b border-slate-700">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="p-2 bg-gradient-to-br from-blue-500 to-purple-500 rounded-lg">
              <Bot className="w-5 h-5 text-white" />
            </div>
            <div>
              <CardTitle className="text-white">AI Tutor</CardTitle>
              <p className="text-xs text-slate-400">Powered by LLM</p>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Badge variant={mode === 'socratic' ? 'default' : 'outline'} className="cursor-pointer" onClick={() => setMode('socratic')}>Socratic</Badge>
            <Badge variant={mode === 'explanation' ? 'default' : 'outline'} className="cursor-pointer" onClick={() => setMode('explanation')}>Explanation</Badge>
            <Badge variant={mode === 'coding' ? 'default' : 'outline'} className="cursor-pointer" onClick={() => setMode('coding')}>Coding</Badge>
          </div>
        </div>
      </CardHeader>

      {/* Message History */}
      <ScrollArea className="flex-1 p-4">
        <div className="space-y-4">
          <AnimatePresence>
            {messages.map((message) => (
              <motion.div
                key={message.id}
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, scale: 0.95 }}
                transition={{ duration: 0.2 }}
                className={`flex gap-3 ${message.role === 'user' ? 'justify-end' : 'justify-start'}`}
              >
                {/* Avatar */}
                <div className={`p-2 rounded-full flex-shrink-0 ${
                  message.role === 'user' 
                    ? 'bg-gradient-to-br from-purple-500 to-pink-500' 
                    : message.role === 'system'
                    ? 'bg-slate-600'
                    : 'bg-gradient-to-br from-blue-500 to-cyan-500'
                }`}>
                  {message.role === 'user' && <User className="w-4 h-4 text-white" />}
                  {message.role === 'system' && <AlertCircle className="w-4 h-4 text-white" />}
                  {message.role === 'assistant' && <Sparkles className="w-4 h-4 text-white" />}
                </div>

                {/* Content Bubble */}
                <div className={`max-w-[80%] p-3 rounded-lg ${
                  message.role === 'user'
                    ? 'bg-purple-500/20 border-purple-500/50'
                    : 'bg-slate-700/50 border-slate-600'
                }`}>
                  {message.type === 'code' ? (
                    <pre className="text-sm font-mono text-green-400 overflow-x-auto">
                      {message.content}
                    </pre>
                  ) : (
                    <p className="text-sm text-slate-200">{message.content}</p>
                  )}
                  <div className="text-xs text-slate-500 mt-1">
                    {new Date(message.timestamp).toLocaleTimeString()}
                  </div>
                </div>
              </motion.div>
            ))}
          </AnimatePresence>
          {isProcessing && (
            <motion.div
              initial={{ opacity: 0, scale: 0.9 }}
              animate={{ opacity: 1, scale: 1 }}
              className="flex gap-3 justify-start"
            >
              <div className="p-2 rounded-full bg-slate-600">
                <Loader2 className="w-4 h-4 text-white animate-spin" />
              </div>
              <div className="p-3 rounded-lg bg-slate-700/50 border border-slate-600">
                <p className="text-sm text-slate-400">Thinking...</p>
              </div>
            </motion.div>
          )}
        </div>
        <div ref={messagesEndRef} />
      </ScrollArea>

      {/* Input Area */}
      <CardContent className="pt-4 border-t border-slate-700">
        <div className="flex gap-2">
          <Input
            placeholder="Ask about ROS2, Kinematics, or generate code..."
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={(e) => e.key === 'Enter' && handleSend()}
            disabled={isProcessing}
            className="flex-1 bg-slate-800/50 border-slate-600 text-white"
          />
          <Button 
            onClick={handleSend} 
            disabled={isProcessing || !input.trim()}
            size="icon"
            className="bg-blue-500 hover:bg-blue-600 text-white"
          >
            <Send className="w-4 h-4" />
          </Button>
        </div>
      </CardContent>
    </div>
  )
}

