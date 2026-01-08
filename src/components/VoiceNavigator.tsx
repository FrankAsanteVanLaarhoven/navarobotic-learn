'use client'

import { useState, useEffect, useRef } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { useRouter, usePathname } from 'next/navigation'
import { Mic, MicOff, Volume2, X, Navigation, Sparkles, Settings } from 'lucide-react'
import { getVoiceNavigationEngine, VoiceCommand } from '@/lib/voice-navigation'
import { cn } from '@/lib/utils'
import { VoiceSettings } from './VoiceSettings'

export function VoiceNavigator() {
  const [isActive, setIsActive] = useState(false)
  const [isListening, setIsListening] = useState(false)
  const [transcript, setTranscript] = useState('')
  const [isProcessing, setIsProcessing] = useState(false)
  const [showHelp, setShowHelp] = useState(false)
  const [showVoiceSettings, setShowVoiceSettings] = useState(false)
  const router = useRouter()
  const pathname = usePathname()
  const engineRef = useRef(getVoiceNavigationEngine())
  const animationRef = useRef<number>()

  // Load voice settings on mount
  useEffect(() => {
    const engine = engineRef.current
    engine.loadVoiceSettings()
  }, [])

  useEffect(() => {
    const engine = engineRef.current

    const handleCommand = (command: VoiceCommand | null, transcriptText: string) => {
      setTranscript(transcriptText)
      setIsProcessing(true)

      if (!command) {
        engine.speak("I didn't understand that. Try saying 'help' for available commands.")
        setTimeout(() => {
          setIsProcessing(false)
          setTranscript('')
        }, 2000)
        return
      }

      // Execute command
      executeCommand(command)
    }

    // Sync with engine state and set up command handler
    const checkState = () => {
      const engineActive = engine.isVoiceActive()
      const engineListening = engine.isCurrentlyListening()
      
      if (engineActive !== isActive) {
        setIsActive(engineActive)
        if (engineActive) {
          // Engine activated, start listening with command handler
          engine.startListening(handleCommand)
        }
      }
      if (engineListening !== isListening) {
        setIsListening(engineListening)
      }
    }

    const interval = setInterval(checkState, 100)

    return () => {
      clearInterval(interval)
    }
  }, [isActive, isListening])

  const executeCommand = (command: VoiceCommand) => {
    const engine = engineRef.current

    switch (command.action) {
      case 'navigate':
        if (command.target) {
          engine.speak(`Navigating to ${command.target}`)
          setTimeout(() => {
            router.push(command.target!)
            setIsProcessing(false)
            setTranscript('')
          }, 1000)
        }
        break

      case 'scroll':
        const scrollAmount = 300
        const scrollDirection = command.direction === 'up' ? -scrollAmount : 
                              command.direction === 'down' ? scrollAmount :
                              command.direction === 'left' ? -scrollAmount : scrollAmount
        
        if (command.direction === 'up' || command.direction === 'down') {
          window.scrollBy({ top: scrollDirection, behavior: 'smooth' })
        } else {
          window.scrollBy({ left: scrollDirection, behavior: 'smooth' })
        }
        
        engine.speak(`Scrolling ${command.direction}`)
        setTimeout(() => {
          setIsProcessing(false)
          setTranscript('')
        }, 1500)
        break

      case 'search':
        engine.speak(`Searching for ${command.query}`)
        // Implement search functionality
        setTimeout(() => {
          setIsProcessing(false)
          setTranscript('')
        }, 2000)
        break

      case 'help':
        setShowHelp(true)
        engine.speak('Here are available commands: Navigate to any page, scroll up or down, search for content, go back, or go home.')
        setTimeout(() => {
          setShowHelp(false)
          setIsProcessing(false)
          setTranscript('')
        }, 8000)
        break

      case 'back':
        engine.speak('Going back')
        router.back()
        setTimeout(() => {
          setIsProcessing(false)
          setTranscript('')
        }, 1000)
        break

      case 'home':
        engine.speak('Going to home page')
        router.push('/')
        setTimeout(() => {
          setIsProcessing(false)
          setTranscript('')
        }, 1000)
        break

      default:
        setIsProcessing(false)
        setTranscript('')
    }
  }

  const toggleVoiceNavigation = () => {
    const engine = engineRef.current
    if (isActive) {
      engine.deactivate()
      setIsActive(false)
      setIsListening(false)
      setTranscript('')
    } else {
      const handleCommand = (command: VoiceCommand | null, transcriptText: string) => {
        setTranscript(transcriptText)
        setIsProcessing(true)

        if (!command) {
          engine.speak("I didn't understand that. Try saying 'help' for available commands.")
          setTimeout(() => {
            setIsProcessing(false)
            setTranscript('')
          }, 2000)
          return
        }

        executeCommand(command)
      }
      engine.activate(handleCommand)
      setIsActive(true)
    }
  }

  // Pulse animation for listening state
  useEffect(() => {
    if (isListening && isActive) {
      animationRef.current = requestAnimationFrame(function animate() {
        animationRef.current = requestAnimationFrame(animate)
      })
    } else {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
    }
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current)
      }
    }
  }, [isListening, isActive])

  if (!isActive && !isListening) {
    return null // Don't render when inactive
  }

  return (
    <AnimatePresence>
      {isActive && (
        <motion.div
          initial={{ opacity: 0, scale: 0.8, y: 20 }}
          animate={{ opacity: 1, scale: 1, y: 0 }}
          exit={{ opacity: 0, scale: 0.8, y: 20 }}
          className="fixed bottom-8 right-8 z-[100] pointer-events-none"
        >
          {/* Main Voice Assistant UI - Dola/Alexa Style */}
          <motion.div
            className={cn(
              "relative w-80 bg-gradient-to-br from-cyan-500/20 via-teal-500/20 to-blue-500/20",
              "backdrop-blur-xl border border-cyan-500/30 rounded-3xl p-6",
              "shadow-2xl pointer-events-auto",
              "before:absolute before:inset-0 before:rounded-3xl",
              "before:bg-gradient-to-br before:from-cyan-500/10 before:via-transparent before:to-blue-500/10",
              "before:pointer-events-none"
            )}
            animate={{
              boxShadow: isListening
                ? [
                    '0 0 20px rgba(6, 182, 212, 0.4)',
                    '0 0 40px rgba(6, 182, 212, 0.6)',
                    '0 0 20px rgba(6, 182, 212, 0.4)',
                  ]
                : '0 0 20px rgba(6, 182, 212, 0.3)',
            }}
            transition={{
              duration: 2,
              repeat: isListening ? Infinity : 0,
              ease: 'easeInOut',
            }}
          >
            {/* Action Buttons */}
            <div className="absolute top-3 right-3 flex items-center gap-2 z-10">
              <button
                onClick={() => setShowVoiceSettings(true)}
                className="w-8 h-8 flex items-center justify-center rounded-full bg-cyan-500/20 hover:bg-cyan-500/30 transition-colors"
                title="Voice Settings"
              >
                <Settings className="w-4 h-4 text-cyan-300" />
              </button>
              <button
                onClick={toggleVoiceNavigation}
                className="w-8 h-8 flex items-center justify-center rounded-full bg-cyan-500/20 hover:bg-cyan-500/30 transition-colors"
                title="Close"
              >
                <X className="w-4 h-4 text-cyan-300" />
              </button>
            </div>

            {/* Voice Indicator Circle */}
            <div className="flex items-center justify-center mb-4">
              <motion.div
                className={cn(
                  "relative w-24 h-24 rounded-full flex items-center justify-center",
                  "bg-gradient-to-br from-cyan-500/30 to-teal-500/30",
                  "border-2 border-cyan-400/50"
                )}
                animate={{
                  scale: isListening ? [1, 1.1, 1] : 1,
                }}
                transition={{
                  duration: 1.5,
                  repeat: isListening ? Infinity : 0,
                  ease: 'easeInOut',
                }}
              >
                {/* Pulsing rings when listening */}
                <AnimatePresence>
                  {isListening && (
                    <>
                      {[0, 1, 2].map((i) => (
                        <motion.div
                          key={i}
                          className="absolute inset-0 rounded-full border-2 border-cyan-400"
                          initial={{ scale: 1, opacity: 0.8 }}
                          animate={{
                            scale: [1, 1.5, 2],
                            opacity: [0.8, 0.4, 0],
                          }}
                          transition={{
                            duration: 2,
                            repeat: Infinity,
                            delay: i * 0.3,
                            ease: 'easeOut',
                          }}
                        />
                      ))}
                    </>
                  )}
                </AnimatePresence>

                {/* Microphone Icon */}
                <motion.div
                  animate={{
                    scale: isListening ? [1, 1.1, 1] : 1,
                  }}
                  transition={{
                    duration: 0.5,
                    repeat: isListening ? Infinity : 0,
                  }}
                >
                  {isListening ? (
                    <Mic className="w-10 h-10 text-cyan-300" />
                  ) : (
                    <MicOff className="w-10 h-10 text-cyan-400/60" />
                  )}
                </motion.div>
              </motion.div>
            </div>

            {/* Status Text */}
            <div className="text-center mb-4">
              <motion.p
                className="text-sm font-medium text-cyan-200 mb-1"
                animate={{
                  opacity: isListening ? [0.7, 1, 0.7] : 0.7,
                }}
                transition={{
                  duration: 1.5,
                  repeat: isListening ? Infinity : 0,
                }}
              >
                {isListening ? 'Listening...' : 'Voice Navigation Active'}
              </motion.p>

              {/* Transcript Display */}
              <AnimatePresence>
                {transcript && (
                  <motion.p
                    initial={{ opacity: 0, y: 10 }}
                    animate={{ opacity: 1, y: 0 }}
                    exit={{ opacity: 0, y: -10 }}
                    className="text-xs text-cyan-300/80 mt-2 min-h-[20px]"
                  >
                    "{transcript}"
                  </motion.p>
                )}
              </AnimatePresence>

              {/* Processing Indicator */}
              {isProcessing && (
                <motion.div
                  initial={{ opacity: 0 }}
                  animate={{ opacity: 1 }}
                  exit={{ opacity: 0 }}
                  className="flex items-center justify-center gap-2 mt-2"
                >
                  <Sparkles className="w-4 h-4 text-cyan-400 animate-pulse" />
                  <span className="text-xs text-cyan-300/60">Processing...</span>
                </motion.div>
              )}
            </div>

            {/* Help Panel */}
            <AnimatePresence>
              {showHelp && (
                <motion.div
                  initial={{ opacity: 0, height: 0 }}
                  animate={{ opacity: 1, height: 'auto' }}
                  exit={{ opacity: 0, height: 0 }}
                  className="mt-4 pt-4 border-t border-cyan-500/20"
                >
                  <p className="text-xs font-semibold text-cyan-200 mb-2">Available Commands:</p>
                  <ul className="text-xs text-cyan-300/70 space-y-1">
                    <li>• "Go to [page name]"</li>
                    <li>• "Scroll up/down"</li>
                    <li>• "Search for [query]"</li>
                    <li>• "Go back" or "Home"</li>
                    <li>• "Help" for more info</li>
                  </ul>
                </motion.div>
              )}
            </AnimatePresence>

            {/* Ephemeral Particles Effect */}
            <div className="absolute inset-0 overflow-hidden rounded-3xl pointer-events-none">
              {isListening && (
                <>
                  {[...Array(6)].map((_, i) => (
                    <motion.div
                      key={i}
                      className="absolute w-1 h-1 bg-cyan-400 rounded-full"
                      initial={{
                        x: '50%',
                        y: '50%',
                        opacity: 0,
                      }}
                      animate={{
                        x: `${50 + (Math.random() - 0.5) * 100}%`,
                        y: `${50 + (Math.random() - 0.5) * 100}%`,
                        opacity: [0, 1, 0],
                        scale: [0, 1, 0],
                      }}
                      transition={{
                        duration: 2,
                        repeat: Infinity,
                        delay: i * 0.3,
                        ease: 'easeOut',
                      }}
                    />
                  ))}
                </>
              )}
            </div>
          </motion.div>
        </motion.div>
      )}
      
      {/* Voice Settings Modal */}
      <VoiceSettings isOpen={showVoiceSettings} onClose={() => setShowVoiceSettings(false)} />
    </AnimatePresence>
  )
}
