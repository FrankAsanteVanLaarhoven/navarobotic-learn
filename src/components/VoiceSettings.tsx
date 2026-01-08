'use client'

import { useState, useEffect } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { Settings, Volume2, Globe, User, Check, X } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Label } from '@/components/ui/label'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select'
import { Switch } from '@/components/ui/switch'
import { Slider } from '@/components/ui/slider'
import { getVoiceNavigationEngine } from '@/lib/voice-navigation'
import { ELEVENLABS_VOICES, ElevenLabsVoiceService, getBrowserVoices, filterBrowserVoices, getBestBrowserVoice } from '@/lib/elevenlabs-voice'
import { cn } from '@/lib/utils'

interface VoiceSettingsProps {
  isOpen: boolean
  onClose: () => void
}

export function VoiceSettings({ isOpen, onClose }: VoiceSettingsProps) {
  const [provider, setProvider] = useState<'elevenlabs' | 'browser'>('browser')
  const [gender, setGender] = useState<'male' | 'female' | undefined>(undefined)
  const [accent, setAccent] = useState<'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'>('us')
  const [selectedElevenLabsVoice, setSelectedElevenLabsVoice] = useState<string>('')
  const [selectedBrowserVoice, setSelectedBrowserVoice] = useState<SpeechSynthesisVoice | null>(null)
  const [browserVoices, setBrowserVoices] = useState<SpeechSynthesisVoice[]>([])
  const [elevenLabsConfigured, setElevenLabsConfigured] = useState(false)
  const [stability, setStability] = useState([0.5])
  const [similarityBoost, setSimilarityBoost] = useState([0.75])
  const [useSpeakerBoost, setUseSpeakerBoost] = useState(true)

  useEffect(() => {
    // Load saved settings
    const saved = localStorage.getItem('voice-navigation-settings')
    if (saved) {
      try {
        const settings = JSON.parse(saved)
        setProvider(settings.provider || 'browser')
        setGender(settings.gender)
        setAccent(settings.accent || 'us')
        setSelectedElevenLabsVoice(settings.voiceId || '')
        setStability(settings.stability !== undefined ? [settings.stability] : [0.5])
        setSimilarityBoost(settings.similarityBoost !== undefined ? [settings.similarityBoost] : [0.75])
        setUseSpeakerBoost(settings.useSpeakerBoost !== undefined ? settings.useSpeakerBoost : true)
      } catch (e) {
        console.warn('Failed to load voice settings')
      }
    }

    // Check ElevenLabs configuration
    const service = new ElevenLabsVoiceService()
    setElevenLabsConfigured(service.isConfigured())

    // Load browser voices
    const loadBrowserVoices = () => {
      const voices = getBrowserVoices()
      setBrowserVoices(voices)
      
      // Set default browser voice
      if (!selectedBrowserVoice && voices.length > 0) {
        const bestVoice = getBestBrowserVoice(gender, accent)
        setSelectedBrowserVoice(bestVoice)
      }
    }

    loadBrowserVoices()
    
    // Some browsers load voices asynchronously
    if ('speechSynthesis' in window) {
      window.speechSynthesis.onvoiceschanged = loadBrowserVoices
    }
  }, [gender, accent])

  const handleSave = () => {
    const engine = getVoiceNavigationEngine()
    
    const settings: any = {
      provider,
      accent,
      gender,
      stability: stability[0],
      similarityBoost: similarityBoost[0],
      useSpeakerBoost,
    }

    if (provider === 'elevenlabs' && selectedElevenLabsVoice) {
      settings.voiceId = selectedElevenLabsVoice
    }

    if (provider === 'browser' && selectedBrowserVoice) {
      settings.browserVoice = selectedBrowserVoice
    }

    engine.setVoiceSettings(settings)
    
    // Test voice
    engine.speak('Voice settings saved. Testing new voice.')
    
    onClose()
  }

  const handleTestVoice = () => {
    const engine = getVoiceNavigationEngine()
    const testText = `Hello, this is a test of the ${gender || 'default'} ${accent} voice. How does this sound?`
    engine.speak(testText)
  }

  const filteredElevenLabsVoices = ELEVENLABS_VOICES.filter(voice => {
    if (gender && voice.gender !== gender) return false
    if (accent && voice.accent !== accent) return false
    return true
  })

  const filteredBrowserVoices = filterBrowserVoices(browserVoices, gender, accent)

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            onClick={onClose}
            className="fixed inset-0 bg-black/50 backdrop-blur-sm z-[200]"
          />
          
          {/* Settings Panel */}
          <motion.div
            initial={{ opacity: 0, scale: 0.9, y: 20 }}
            animate={{ opacity: 1, scale: 1, y: 0 }}
            exit={{ opacity: 0, scale: 0.9, y: 20 }}
            className="fixed top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 z-[201] w-full max-w-2xl max-h-[90vh] overflow-y-auto"
          >
            <Card className="bg-gradient-to-br from-cyan-500/10 via-teal-500/10 to-blue-500/10 backdrop-blur-xl border-cyan-500/30">
              <CardHeader className="border-b border-cyan-500/20">
                <div className="flex items-center justify-between">
                  <CardTitle className="flex items-center gap-2 text-cyan-200">
                    <Settings className="w-5 h-5" />
                    Voice Settings
                  </CardTitle>
                  <Button variant="ghost" size="sm" onClick={onClose}>
                    <X className="w-4 h-4" />
                  </Button>
                </div>
              </CardHeader>
              
              <CardContent className="p-6 space-y-6">
                {/* Provider Selection */}
                <div className="space-y-2">
                  <Label className="text-cyan-200">Voice Provider</Label>
                  <Select value={provider} onValueChange={(v: 'elevenlabs' | 'browser') => setProvider(v)}>
                    <SelectTrigger className="bg-background/50 border-cyan-500/30">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="browser">Browser Native (Free)</SelectItem>
                      <SelectItem value="elevenlabs" disabled={!elevenLabsConfigured}>
                        ElevenLabs {elevenLabsConfigured ? '(Premium)' : '(API Key Required)'}
                      </SelectItem>
                    </SelectContent>
                  </Select>
                  {!elevenLabsConfigured && provider === 'elevenlabs' && (
                    <p className="text-xs text-cyan-300/60">
                      Add NEXT_PUBLIC_ELEVENLABS_API_KEY to .env for ElevenLabs voices
                    </p>
                  )}
                </div>

                {/* Gender Selection */}
                <div className="space-y-2">
                  <Label className="text-cyan-200">Gender</Label>
                  <Select 
                    value={gender || 'any'} 
                    onValueChange={(v) => setGender(v === 'any' ? undefined : v as 'male' | 'female')}
                  >
                    <SelectTrigger className="bg-background/50 border-cyan-500/30">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="any">Any</SelectItem>
                      <SelectItem value="male">Male</SelectItem>
                      <SelectItem value="female">Female</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                {/* Accent Selection */}
                <div className="space-y-2">
                  <Label className="text-cyan-200">Accent</Label>
                  <Select value={accent} onValueChange={(v: any) => setAccent(v)}>
                    <SelectTrigger className="bg-background/50 border-cyan-500/30">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="us">🇺🇸 US English</SelectItem>
                      <SelectItem value="uk">🇬🇧 UK English</SelectItem>
                      <SelectItem value="australia">🇦🇺 Australian</SelectItem>
                      <SelectItem value="canada">🇨🇦 Canadian</SelectItem>
                      <SelectItem value="irish">🇮🇪 Irish</SelectItem>
                      <SelectItem value="scottish">🏴󠁧󠁢󠁳󠁣󠁴󠁿 Scottish</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                {/* Voice Selection */}
                {provider === 'elevenlabs' && (
                  <div className="space-y-2">
                    <Label className="text-cyan-200">ElevenLabs Voice</Label>
                    <Select value={selectedElevenLabsVoice} onValueChange={setSelectedElevenLabsVoice}>
                      <SelectTrigger className="bg-background/50 border-cyan-500/30">
                        <SelectValue placeholder="Select a voice" />
                      </SelectTrigger>
                      <SelectContent>
                        {filteredElevenLabsVoices.map((voice) => (
                          <SelectItem key={voice.voice_id} value={voice.voice_id}>
                            <div className="flex items-center gap-2">
                              <span>{voice.name}</span>
                              <span className="text-xs text-muted-foreground">
                                ({voice.gender}, {voice.accent})
                              </span>
                            </div>
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                    <p className="text-xs text-cyan-300/60">{filteredElevenLabsVoices.length} voices available</p>
                  </div>
                )}

                {provider === 'browser' && (
                  <div className="space-y-2">
                    <Label className="text-cyan-200">Browser Voice</Label>
                    <Select 
                      value={selectedBrowserVoice?.name || ''} 
                      onValueChange={(name) => {
                        const voice = browserVoices.find(v => v.name === name)
                        setSelectedBrowserVoice(voice || null)
                      }}
                    >
                      <SelectTrigger className="bg-background/50 border-cyan-500/30">
                        <SelectValue placeholder="Select a voice" />
                      </SelectTrigger>
                      <SelectContent>
                        {filteredBrowserVoices.map((voice) => (
                          <SelectItem key={voice.name} value={voice.name}>
                            <div className="flex items-center gap-2">
                              <span>{voice.name}</span>
                              <span className="text-xs text-muted-foreground">({voice.lang})</span>
                            </div>
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                    <p className="text-xs text-cyan-300/60">{filteredBrowserVoices.length} voices available</p>
                  </div>
                )}

                {/* ElevenLabs Advanced Settings */}
                {provider === 'elevenlabs' && (
                  <div className="space-y-4 pt-4 border-t border-cyan-500/20">
                    <div className="space-y-2">
                      <Label className="text-cyan-200">Stability: {stability[0].toFixed(2)}</Label>
                      <Slider
                        value={stability}
                        onValueChange={setStability}
                        min={0}
                        max={1}
                        step={0.1}
                        className="w-full"
                      />
                      <p className="text-xs text-cyan-300/60">Controls voice consistency</p>
                    </div>

                    <div className="space-y-2">
                      <Label className="text-cyan-200">Similarity Boost: {similarityBoost[0].toFixed(2)}</Label>
                      <Slider
                        value={similarityBoost}
                        onValueChange={setSimilarityBoost}
                        min={0}
                        max={1}
                        step={0.1}
                        className="w-full"
                      />
                      <p className="text-xs text-cyan-300/60">Controls voice similarity to original</p>
                    </div>

                    <div className="flex items-center justify-between">
                      <Label className="text-cyan-200">Speaker Boost</Label>
                      <Switch
                        checked={useSpeakerBoost}
                        onCheckedChange={setUseSpeakerBoost}
                      />
                    </div>
                  </div>
                )}

                {/* Action Buttons */}
                <div className="flex gap-3 pt-4 border-t border-cyan-500/20">
                  <Button
                    variant="outline"
                    onClick={handleTestVoice}
                    className="flex-1 border-cyan-500/30 text-cyan-200 hover:bg-cyan-500/20"
                  >
                    <Volume2 className="w-4 h-4 mr-2" />
                    Test Voice
                  </Button>
                  <Button
                    onClick={handleSave}
                    className="flex-1 bg-cyan-500 hover:bg-cyan-600 text-white"
                  >
                    <Check className="w-4 h-4 mr-2" />
                    Save Settings
                  </Button>
                </div>
              </CardContent>
            </Card>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  )
}
