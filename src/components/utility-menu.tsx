'use client'

import { useState, useEffect } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { Button } from '@/components/ui/button'
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select'
import { Input } from '@/components/ui/input'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import {
  Globe,
  Clock,
  DollarSign,
  Cloud,
  Moon,
  Sun,
  X,
  Languages,
  Calculator,
  Calendar,
  Volume2,
  VolumeX,
  Settings,
  Mic
} from 'lucide-react'
import { useTheme } from 'next-themes'
import { getSoundManager } from '@/lib/sounds'
import { Label } from '@/components/ui/label'
import { Switch } from '@/components/ui/switch'
import { getVoiceNavigationEngine } from '@/lib/voice-navigation'
import { ELEVENLABS_VOICES, ElevenLabsVoiceService, getBrowserVoices, filterBrowserVoices, getBestBrowserVoice } from '@/lib/elevenlabs-voice'
import { Slider } from '@/components/ui/slider'

interface WorldClock {
  city: string
  timezone: string
  time: string
}

interface WeatherDay {
  day: string
  date: string
  high: number
  low: number
  condition: string
  icon: string
}

export function UtilityMenu() {
  const [isOpen, setIsOpen] = useState(false)
  const [selectedLanguage, setSelectedLanguage] = useState('en')
  const [clocks, setClocks] = useState<WorldClock[]>([])
  const [fromCurrency, setFromCurrency] = useState('USD')
  const [toCurrency, setToCurrency] = useState('EUR')
  const [amount, setAmount] = useState('1')
  const [convertedAmount, setConvertedAmount] = useState('0.85')
  const [weather, setWeather] = useState<WeatherDay[]>([])
  const [location, setLocation] = useState('New York, US')
  const { theme, setTheme } = useTheme()
  const [mounted, setMounted] = useState(false)
  const [soundEnabled, setSoundEnabled] = useState(true)
  const [soundVolume, setSoundVolume] = useState(30)
  const [voiceProvider, setVoiceProvider] = useState<'elevenlabs' | 'browser'>('browser')
  const [voiceGender, setVoiceGender] = useState<'male' | 'female' | undefined>(undefined)
  const [voiceAccent, setVoiceAccent] = useState<'us' | 'uk' | 'australia' | 'canada' | 'irish' | 'scottish'>('us')
  const [selectedElevenLabsVoice, setSelectedElevenLabsVoice] = useState<string>('')
  const [selectedBrowserVoice, setSelectedBrowserVoice] = useState<SpeechSynthesisVoice | null>(null)
  const [browserVoices, setBrowserVoices] = useState<SpeechSynthesisVoice[]>([])
  const [elevenLabsConfigured, setElevenLabsConfigured] = useState(false)
  const [voiceStability, setVoiceStability] = useState([0.5])
  const [voiceSimilarityBoost, setVoiceSimilarityBoost] = useState([0.75])
  const [useSpeakerBoost, setUseSpeakerBoost] = useState(true)

  useEffect(() => {
    setMounted(true)
    // Load sound preference from localStorage
    const savedSoundPreference = localStorage.getItem('sound-enabled')
    if (savedSoundPreference !== null) {
      const enabled = savedSoundPreference === 'true'
      setSoundEnabled(enabled)
      getSoundManager().setEnabled(enabled)
    }
    // Load volume preference
    const savedVolume = localStorage.getItem('sound-volume')
    if (savedVolume !== null) {
      const volume = parseFloat(savedVolume)
      setSoundVolume(Math.round(volume * 100))
      getSoundManager().setVolume(volume)
    }

    // Load voice settings
    const savedVoiceSettings = localStorage.getItem('voice-navigation-settings')
    if (savedVoiceSettings) {
      try {
        const settings = JSON.parse(savedVoiceSettings)
        setVoiceProvider(settings.provider || 'browser')
        setVoiceGender(settings.gender)
        setVoiceAccent(settings.accent || 'us')
        setSelectedElevenLabsVoice(settings.voiceId || '')
        setVoiceStability(settings.stability !== undefined ? [settings.stability] : [0.5])
        setVoiceSimilarityBoost(settings.similarityBoost !== undefined ? [settings.similarityBoost] : [0.75])
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
      if (!selectedBrowserVoice && voices.length > 0) {
        const bestVoice = getBestBrowserVoice(voiceGender, voiceAccent)
        setSelectedBrowserVoice(bestVoice)
      }
    }

    loadBrowserVoices()
    if ('speechSynthesis' in window) {
      window.speechSynthesis.onvoiceschanged = loadBrowserVoices
    }
  }, [])

  useEffect(() => {
    if (isOpen) {
      updateClocks()
      updateWeather()
      const clockInterval = setInterval(updateClocks, 1000)
      return () => clearInterval(clockInterval)
    }
  }, [isOpen])

  const updateClocks = () => {
    const cities = [
      { city: 'New York', timezone: 'America/New_York' },
      { city: 'London', timezone: 'Europe/London' },
      { city: 'Tokyo', timezone: 'Asia/Tokyo' },
      { city: 'Sydney', timezone: 'Australia/Sydney' },
      { city: 'San Francisco', timezone: 'America/Los_Angeles' },
    ]

    const newClocks = cities.map(({ city, timezone }) => {
      const now = new Date()
      const timeString = new Intl.DateTimeFormat('en-US', {
        timeZone: timezone,
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
        hour12: true,
      }).format(now)

      return { city, timezone, time: timeString }
    })

    setClocks(newClocks)
  }

  const updateWeather = () => {
    // Mock weather data - replace with real API call
    const days = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
    const conditions = ['Sunny', 'Cloudy', 'Rainy', 'Partly Cloudy']
    const icons = ['☀️', '☁️', '🌧️', '⛅']

    const weatherData: WeatherDay[] = days.map((day, index) => {
      const date = new Date()
      date.setDate(date.getDate() + index)
      return {
        day,
        date: date.toLocaleDateString('en-US', { month: 'short', day: 'numeric' }),
        high: Math.floor(Math.random() * 15) + 20,
        low: Math.floor(Math.random() * 10) + 10,
        condition: conditions[Math.floor(Math.random() * conditions.length)],
        icon: icons[Math.floor(Math.random() * icons.length)],
      }
    })

    setWeather(weatherData)
  }

  const handleCurrencyConvert = () => {
    // Mock conversion - replace with real API call
    const rates: Record<string, Record<string, number>> = {
      USD: { EUR: 0.85, GBP: 0.73, JPY: 110, CNY: 6.5 },
      EUR: { USD: 1.18, GBP: 0.86, JPY: 129, CNY: 7.65 },
      GBP: { USD: 1.37, EUR: 1.16, JPY: 150, CNY: 8.9 },
      JPY: { USD: 0.0091, EUR: 0.0077, GBP: 0.0067, CNY: 0.059 },
      CNY: { USD: 0.15, EUR: 0.13, GBP: 0.11, JPY: 16.9 },
    }

    const rate = rates[fromCurrency]?.[toCurrency] || 1
    const result = (parseFloat(amount) * rate).toFixed(2)
    setConvertedAmount(result)
  }

  useEffect(() => {
    handleCurrencyConvert()
  }, [fromCurrency, toCurrency, amount])

  const languages = [
    { code: 'en', name: 'English', flag: '🇺🇸' },
    { code: 'es', name: 'Español', flag: '🇪🇸' },
    { code: 'fr', name: 'Français', flag: '🇫🇷' },
    { code: 'de', name: 'Deutsch', flag: '🇩🇪' },
    { code: 'ja', name: '日本語', flag: '🇯🇵' },
    { code: 'zh', name: '中文', flag: '🇨🇳' },
  ]

  if (!mounted) return null

  return (
    <DropdownMenu open={isOpen} onOpenChange={setIsOpen}>
      <DropdownMenuTrigger asChild>
        <Button
          variant="ghost"
          size="sm"
          className="relative glass rounded-xl border-border/50 hover:border-primary/50 transition-all"
        >
          <Globe className="w-4 h-4 mr-2" />
          <span className="hidden sm:inline">Utilities</span>
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent
        align="end"
        className="w-[90vw] sm:w-[500px] p-0 glass border-border/50 rounded-xl overflow-hidden"
      >
        <div className="p-4 border-b border-border/50 flex items-center justify-between">
          <h3 className="font-semibold text-lg">Utilities</h3>
          <div className="flex items-center gap-2">
            <Button
              variant="ghost"
              size="sm"
              onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}
              className="h-8 w-8 p-0 transition-all duration-200 hover:bg-accent/50 hover:scale-110 hover:rotate-180"
            >
              {theme === 'dark' ? (
                <Sun className="w-4 h-4 transition-transform duration-200" />
              ) : (
                <Moon className="w-4 h-4 transition-transform duration-200" />
              )}
            </Button>
            <Button
              variant="ghost"
              size="sm"
              onClick={() => setIsOpen(false)}
              className="h-8 w-8 p-0 transition-all duration-200 hover:bg-destructive/20 hover:scale-110 hover:rotate-90"
            >
              <X className="w-4 h-4 transition-transform duration-200" />
            </Button>
          </div>
        </div>

        <Tabs defaultValue="language" className="w-full">
          <TabsList className="w-full rounded-none border-b border-border/50 bg-transparent h-auto p-0 grid grid-cols-6">
            <TabsTrigger 
              value="language" 
              className="flex-1 rounded-none data-[state=active]:border-b-2 data-[state=active]:border-primary transition-all duration-200 hover:bg-accent/50 hover:scale-105 group"
            >
              <Languages className="w-4 h-4 mr-1 transition-transform duration-200 group-hover:scale-110" />
              <span className="hidden sm:inline">Language</span>
            </TabsTrigger>
            <TabsTrigger 
              value="clock" 
              className="flex-1 rounded-none data-[state=active]:border-b-2 data-[state=active]:border-primary transition-all duration-200 hover:bg-accent/50 hover:scale-105 group"
            >
              <Clock className="w-4 h-4 mr-1 transition-transform duration-200 group-hover:rotate-12" />
              <span className="hidden sm:inline">Clock</span>
            </TabsTrigger>
            <TabsTrigger 
              value="currency" 
              className="flex-1 rounded-none data-[state=active]:border-b-2 data-[state=active]:border-primary transition-all duration-200 hover:bg-accent/50 hover:scale-105 group"
            >
              <DollarSign className="w-4 h-4 mr-1 transition-transform duration-200 group-hover:scale-110" />
              <span className="hidden sm:inline">Currency</span>
            </TabsTrigger>
            <TabsTrigger 
              value="weather" 
              className="flex-1 rounded-none data-[state=active]:border-b-2 data-[state=active]:border-primary transition-all duration-200 hover:bg-accent/50 hover:scale-105 group"
            >
              <Cloud className="w-4 h-4 mr-1 transition-transform duration-200 group-hover:scale-110" />
              <span className="hidden sm:inline">Weather</span>
            </TabsTrigger>
            <TabsTrigger 
              value="voice" 
              className="flex-1 rounded-none data-[state=active]:border-b-2 data-[state=active]:border-primary transition-all duration-200 hover:bg-accent/50 hover:scale-105 group"
            >
              <Mic className="w-4 h-4 mr-1 transition-transform duration-200 group-hover:scale-110" />
              <span className="hidden sm:inline">Voice</span>
            </TabsTrigger>
            <TabsTrigger 
              value="settings" 
              className="flex-1 rounded-none data-[state=active]:border-b-2 data-[state=active]:border-primary transition-all duration-200 hover:bg-accent/50 hover:scale-105 group"
            >
              <Settings className="w-4 h-4 mr-1 transition-transform duration-200 group-hover:scale-110" />
              <span className="hidden sm:inline">Settings</span>
            </TabsTrigger>
          </TabsList>

          <div className="max-h-[500px] overflow-y-auto">
            <TabsContent value="language" className="p-4 m-0">
              <div className="space-y-4">
                <div>
                  <label className="text-sm font-medium mb-2 block">Select Language</label>
                  <Select value={selectedLanguage} onValueChange={setSelectedLanguage}>
                    <SelectTrigger className="w-full transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {languages.map((lang) => (
                        <SelectItem key={lang.code} value={lang.code}>
                          <span className="mr-2">{lang.flag}</span>
                          {lang.name}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
                <Card className="border-border/50 transition-all duration-200 hover:border-primary/50 hover:shadow-md hover:scale-[1.02]">
                  <CardHeader className="pb-3">
                    <CardTitle className="text-sm">Current Language</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <div className="flex items-center gap-3">
                      <span className="text-2xl transition-transform duration-200 hover:scale-125">
                        {languages.find(l => l.code === selectedLanguage)?.flag}
                      </span>
                      <div>
                        <div className="font-medium">
                          {languages.find(l => l.code === selectedLanguage)?.name}
                        </div>
                        <div className="text-xs text-muted-foreground">
                          Language code: {selectedLanguage.toUpperCase()}
                        </div>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            <TabsContent value="clock" className="p-4 m-0">
              <div className="space-y-3">
                {clocks.map((clock, index) => (
                  <Card key={index} className="border-border/50 transition-all duration-200 hover:border-primary/50 hover:shadow-md hover:scale-[1.02] hover:bg-accent/5">
                    <CardContent className="p-4">
                      <div className="flex items-center justify-between">
                        <div>
                          <div className="font-semibold">{clock.city}</div>
                          <div className="text-xs text-muted-foreground">{clock.timezone}</div>
                        </div>
                        <div className="text-right">
                          <div className="text-2xl font-mono font-bold transition-all duration-200 hover:scale-110">{clock.time}</div>
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </TabsContent>

            <TabsContent value="currency" className="p-4 m-0">
              <div className="space-y-4">
                <div className="grid grid-cols-2 gap-3">
                  <div>
                    <label className="text-sm font-medium mb-2 block">From</label>
                    <Select value={fromCurrency} onValueChange={setFromCurrency}>
                      <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="USD">USD ($)</SelectItem>
                        <SelectItem value="EUR">EUR (€)</SelectItem>
                        <SelectItem value="GBP">GBP (£)</SelectItem>
                        <SelectItem value="JPY">JPY (¥)</SelectItem>
                        <SelectItem value="CNY">CNY (¥)</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                  <div>
                    <label className="text-sm font-medium mb-2 block">To</label>
                    <Select value={toCurrency} onValueChange={setToCurrency}>
                      <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        <SelectItem value="USD">USD ($)</SelectItem>
                        <SelectItem value="EUR">EUR (€)</SelectItem>
                        <SelectItem value="GBP">GBP (£)</SelectItem>
                        <SelectItem value="JPY">JPY (¥)</SelectItem>
                        <SelectItem value="CNY">CNY (¥)</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                </div>
                <div>
                  <label className="text-sm font-medium mb-2 block">Amount</label>
                  <Input
                    type="number"
                    value={amount}
                    onChange={(e) => setAmount(e.target.value)}
                    placeholder="Enter amount"
                    className="transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                  />
                </div>
                <Card className="border-border/50 bg-primary/5 transition-all duration-200 hover:border-primary/50 hover:shadow-lg hover:scale-[1.02] hover:bg-primary/10">
                  <CardContent className="p-4">
                    <div className="text-center">
                      <div className="text-sm text-muted-foreground mb-1">Converted Amount</div>
                      <div className="text-3xl font-bold transition-all duration-200 hover:scale-110">
                        {convertedAmount} {toCurrency}
                      </div>
                      <div className="text-xs text-muted-foreground mt-1">
                        1 {fromCurrency} = {(parseFloat(convertedAmount) / parseFloat(amount || '1')).toFixed(4)} {toCurrency}
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            <TabsContent value="weather" className="p-4 m-0">
              <div className="space-y-3">
                <div>
                  <label className="text-sm font-medium mb-2 block">Location</label>
                  <Input
                    value={location}
                    onChange={(e) => setLocation(e.target.value)}
                    placeholder="Enter city name"
                    className="transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                  />
                </div>
                <div className="grid grid-cols-2 sm:grid-cols-4 gap-2">
                  {weather.map((day, index) => (
                    <Card key={index} className="border-border/50 transition-all duration-200 hover:border-primary/50 hover:shadow-md hover:scale-105 hover:bg-accent/5 cursor-pointer group">
                      <CardContent className="p-3 text-center">
                        <div className="text-xs text-muted-foreground mb-1">{day.day}</div>
                        <div className="text-xs text-muted-foreground mb-2">{day.date}</div>
                        <div className="text-2xl mb-2 transition-transform duration-200 group-hover:scale-125 group-hover:rotate-12">{day.icon}</div>
                        <div className="font-semibold transition-all duration-200 group-hover:text-primary">{day.high}°</div>
                        <div className="text-xs text-muted-foreground">{day.low}°</div>
                        <div className="text-xs text-muted-foreground mt-1">{day.condition}</div>
                      </CardContent>
                    </Card>
                  ))}
                </div>
              </div>
            </TabsContent>

            <TabsContent value="voice" className="p-4 m-0">
              <div className="space-y-4">
                <div>
                  <CardTitle className="text-lg mb-4 flex items-center gap-2">
                    <Mic className="w-5 h-5 text-primary" />
                    Voice Navigation Settings
                  </CardTitle>
                </div>

                {/* Provider Selection */}
                <div className="space-y-2">
                  <Label className="text-sm font-medium">Voice Provider</Label>
                  <Select value={voiceProvider} onValueChange={(v: 'elevenlabs' | 'browser') => setVoiceProvider(v)}>
                    <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="browser">Browser Native (Free)</SelectItem>
                      <SelectItem value="elevenlabs" disabled={!elevenLabsConfigured}>
                        ElevenLabs {elevenLabsConfigured ? '(Premium)' : '(API Key Required)'}
                      </SelectItem>
                    </SelectContent>
                  </Select>
                  {!elevenLabsConfigured && voiceProvider === 'elevenlabs' && (
                    <p className="text-xs text-muted-foreground">
                      Add NEXT_PUBLIC_ELEVENLABS_API_KEY to .env for ElevenLabs voices
                    </p>
                  )}
                </div>

                {/* Gender Selection */}
                <div className="space-y-2">
                  <Label className="text-sm font-medium">Gender</Label>
                  <Select 
                    value={voiceGender || 'any'} 
                    onValueChange={(v) => setVoiceGender(v === 'any' ? undefined : v as 'male' | 'female')}
                  >
                    <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
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
                  <Label className="text-sm font-medium">Accent</Label>
                  <Select value={voiceAccent} onValueChange={(v: any) => setVoiceAccent(v)}>
                    <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
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
                {voiceProvider === 'elevenlabs' && (
                  <div className="space-y-2">
                    <Label className="text-sm font-medium">ElevenLabs Voice</Label>
                    <Select value={selectedElevenLabsVoice} onValueChange={setSelectedElevenLabsVoice}>
                      <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
                        <SelectValue placeholder="Select a voice" />
                      </SelectTrigger>
                      <SelectContent>
                        {ELEVENLABS_VOICES.filter(v => {
                          if (voiceGender && v.gender !== voiceGender) return false
                          if (voiceAccent && v.accent !== voiceAccent) return false
                          return true
                        }).map((voice) => (
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
                  </div>
                )}

                {voiceProvider === 'browser' && (
                  <div className="space-y-2">
                    <Label className="text-sm font-medium">Browser Voice</Label>
                    <Select 
                      value={selectedBrowserVoice?.name || ''} 
                      onValueChange={(name) => {
                        const voice = browserVoices.find(v => v.name === name)
                        setSelectedBrowserVoice(voice || null)
                      }}
                    >
                      <SelectTrigger className="transition-all duration-200 hover:border-primary/50 hover:shadow-sm">
                        <SelectValue placeholder="Select a voice" />
                      </SelectTrigger>
                      <SelectContent>
                        {filterBrowserVoices(browserVoices, voiceGender, voiceAccent).map((voice) => (
                          <SelectItem key={voice.name} value={voice.name}>
                            <div className="flex items-center gap-2">
                              <span>{voice.name}</span>
                              <span className="text-xs text-muted-foreground">({voice.lang})</span>
                            </div>
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  </div>
                )}

                {/* ElevenLabs Advanced Settings */}
                {voiceProvider === 'elevenlabs' && (
                  <div className="space-y-4 pt-4 border-t border-border/50">
                    <div className="space-y-2">
                      <Label className="text-sm font-medium">Stability: {voiceStability[0].toFixed(2)}</Label>
                      <Slider
                        value={voiceStability}
                        onValueChange={setVoiceStability}
                        min={0}
                        max={1}
                        step={0.1}
                        className="w-full"
                      />
                      <p className="text-xs text-muted-foreground">Controls voice consistency</p>
                    </div>

                    <div className="space-y-2">
                      <Label className="text-sm font-medium">Similarity Boost: {voiceSimilarityBoost[0].toFixed(2)}</Label>
                      <Slider
                        value={voiceSimilarityBoost}
                        onValueChange={setVoiceSimilarityBoost}
                        min={0}
                        max={1}
                        step={0.1}
                        className="w-full"
                      />
                      <p className="text-xs text-muted-foreground">Controls voice similarity to original</p>
                    </div>

                    <div className="flex items-center justify-between">
                      <Label className="text-sm font-medium">Speaker Boost</Label>
                      <Switch
                        checked={useSpeakerBoost}
                        onCheckedChange={setUseSpeakerBoost}
                      />
                    </div>
                  </div>
                )}

                {/* Action Buttons */}
                <div className="flex gap-3 pt-4 border-t border-border/50">
                  <Button
                    variant="outline"
                    onClick={() => {
                      const engine = getVoiceNavigationEngine()
                      const testText = `Hello, this is a test of the ${voiceGender || 'default'} ${voiceAccent} voice. How does this sound?`
                      engine.speak(testText)
                    }}
                    className="flex-1 transition-all duration-200 hover:bg-accent/50 hover:scale-105"
                  >
                    <Volume2 className="w-4 h-4 mr-2" />
                    Test Voice
                  </Button>
                  <Button
                    onClick={() => {
                      const engine = getVoiceNavigationEngine()
                      const settings: any = {
                        provider: voiceProvider,
                        accent: voiceAccent,
                        gender: voiceGender,
                        stability: voiceStability[0],
                        similarityBoost: voiceSimilarityBoost[0],
                        useSpeakerBoost,
                      }

                      if (voiceProvider === 'elevenlabs' && selectedElevenLabsVoice) {
                        settings.voiceId = selectedElevenLabsVoice
                      }

                      if (voiceProvider === 'browser' && selectedBrowserVoice) {
                        settings.browserVoice = selectedBrowserVoice
                      }

                      engine.setVoiceSettings(settings)
                      engine.speak('Voice settings saved. Testing new voice.')
                    }}
                    className="flex-1 bg-primary hover:bg-primary/90 transition-all duration-200 hover:scale-105"
                  >
                    Save Settings
                  </Button>
                </div>
              </div>
            </TabsContent>

            <TabsContent value="settings" className="p-4 m-0">
              <div className="space-y-6">
                <div>
                  <CardTitle className="text-lg mb-4 flex items-center gap-2">
                    <Settings className="w-5 h-5 text-primary" />
                    Platform Settings
                  </CardTitle>
                </div>

                <div className="space-y-4">
                  <Card className="border-border/50">
                    <CardContent className="p-4">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-3">
                          {soundEnabled ? (
                            <Volume2 className="w-5 h-5 text-primary" />
                          ) : (
                            <VolumeX className="w-5 h-5 text-muted-foreground" />
                          )}
                          <div>
                            <Label htmlFor="sound-toggle" className="text-sm font-medium cursor-pointer">
                              Sound Effects
                            </Label>
                            <p className="text-xs text-muted-foreground mt-1">
                              Enable robotic sound feedback for interactions
                            </p>
                          </div>
                        </div>
                        <Switch
                          id="sound-toggle"
                          checked={soundEnabled}
                          onCheckedChange={(checked) => {
                            setSoundEnabled(checked)
                            getSoundManager().setEnabled(checked)
                            localStorage.setItem('sound-enabled', String(checked))
                            // Play a sound when enabling
                            if (checked) {
                              setTimeout(() => {
                                getSoundManager().playBeep(600, 0.1)
                              }, 100)
                            }
                          }}
                        />
                      </div>
                    </CardContent>
                  </Card>

                  <Card className="border-border/50">
                    <CardContent className="p-4">
                      <div className="space-y-2">
                        <Label className="text-sm font-medium">Sound Volume</Label>
                        <div className="flex items-center gap-3">
                          <VolumeX className="w-4 h-4 text-muted-foreground" />
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={soundVolume}
                            onChange={(e) => {
                              const volume = Number(e.target.value) / 100
                              setSoundVolume(Number(e.target.value))
                              getSoundManager().setVolume(volume)
                              localStorage.setItem('sound-volume', String(volume))
                            }}
                            className="flex-1 h-2 bg-background rounded-lg appearance-none cursor-pointer"
                            disabled={!soundEnabled}
                          />
                          <span className="text-xs text-muted-foreground w-10 text-right">{soundVolume}%</span>
                          <Volume2 className="w-4 h-4 text-muted-foreground" />
                        </div>
                        <p className="text-xs text-muted-foreground">
                          Adjust the volume of sound effects
                        </p>
                      </div>
                    </CardContent>
                  </Card>

                  <div className="text-xs text-muted-foreground pt-2 border-t border-border/50">
                    <p>Sound effects provide audio feedback for clicks, hovers, and selections throughout the platform.</p>
                  </div>
                </div>
              </div>
            </TabsContent>
          </div>
        </Tabs>
      </DropdownMenuContent>
    </DropdownMenu>
  )
}
