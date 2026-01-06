'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Slider } from '@/components/ui/slider'
import { Label } from '@/components/ui/label'
import {
  Mic,
  Volume2,
  Play,
  Pause,
  RefreshCw,
  CheckCircle2,
  ChevronRight,
  Settings,
  Info,
  Star,
  Clock,
  Users
} from 'lucide-react'
import { useState } from 'react'
import Link from 'next/link'

interface VoiceProfile {
  id: string
  name: string
  displayName: string
  gender: string
  accent: string
  age: string
  style: string
  sampleAudioUrl: string
}

export default function VoiceSelectionPage() {
  const [selectedVoice, setSelectedVoice] = useState<string | null>(null)
  const [isPlaying, setIsPlaying] = useState(false)
  const [volume, setVolume] = useState(80)

  const voices: VoiceProfile[] = [
    {
      id: 'voice-adam',
      name: 'Adam',
      displayName: 'Adam',
      gender: 'male',
      accent: 'American',
      age: 'Young',
      style: 'Professional',
      sampleAudioUrl: '/audio/samples/adam.mp3'
    },
    {
      id: 'voice-bella',
      name: 'Bella',
      displayName: 'Bella',
      gender: 'female',
      accent: 'American',
      age: 'Young',
      style: 'Professional',
      sampleAudioUrl: '/audio/samples/bella.mp3'
    },
    {
      id: 'voice-charlie',
      name: 'Charlie',
      displayName: 'Charlie',
      gender: 'male',
      accent: 'British',
      age: 'Middle-aged',
      style: 'Casual',
      sampleAudioUrl: '/audio/samples/charlie.mp3'
    },
    {
      id: 'voice-diana',
      name: 'Diana',
      displayName: 'Diana',
      gender: 'female',
      accent: 'Australian',
      age: 'Middle-aged',
      style: 'Excited',
      sampleAudioUrl: '/audio/samples/diana.mp3'
    },
    {
      id: 'voice-ethan',
      name: 'Ethan',
      displayName: 'Ethan',
      gender: 'male',
      accent: 'American',
      age: 'Elderly',
      style: 'Calm',
      sampleAudioUrl: '/audio/samples/ethan.mp3'
    },
    {
      id: 'voice-nova',
      name: 'Nova',
      displayName: 'Nova',
      gender: 'neutral',
      accent: 'American',
      age: 'Young',
      style: 'Professional',
      sampleAudioUrl: '/audio/samples/nova.mp3'
    },
    {
      id: 'voice-echo',
      name: 'Echo',
      displayName: 'Echo',
      gender: 'female',
      accent: 'British',
      age: 'Young',
      style: 'Professional',
      sampleAudioUrl: '/audio/samples/echo.mp3'
    }
  ]

  const handleVoiceSelect = (voiceId: string) => {
    setSelectedVoice(voiceId)
    // Save user's voice preference
    console.log('Selected voice:', voiceId)
  }

  const getVoiceColor = (voice: VoiceProfile) => {
    switch (voice.gender) {
      case 'male': return 'text-blue-500'
      case 'female': return 'text-pink-500'
      case 'neutral': return 'text-purple-500'
      default: return 'text-gray-500'
    }
  }

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-4">
              <Link href="/student" className="flex items-center gap-2">
                <Mic className="h-8 w-8 text-primary" />
                <span className="text-lg font-bold gradient-text">AI VOICE STUDIO</span>
              </Link>
            </div>

            <div className="flex items-center gap-4">
              <Link href="/ai-video/generate" className="text-sm text-muted-foreground hover:text-foreground transition-colors">
                Skip
              </Link>
              <Button variant="outline" size="sm">
                <Settings className="w-4 h-4 mr-2" />
                Settings
              </Button>
            </div>
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="pt-20 pb-8 px-4">
        <div className="container mx-auto max-w-6xl">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            className="text-center mb-12"
          >
            <h1 className="text-4xl font-bold mb-4">
              Choose Your <span className="gradient-text">AI Voice</span>
            </h1>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              Select a voice for your AI-generated video courses. Our advanced TTS system offers multiple voice profiles like ElevenLabs.
            </p>
          </motion.div>

          <Tabs defaultValue="all" className="w-full">
            <TabsList className="grid w-full max-w-2xl grid-cols-4 mb-8">
              <TabsTrigger value="all">All Voices</TabsTrigger>
              <TabsTrigger value="male">Male</TabsTrigger>
              <TabsTrigger value="female">Female</TabsTrigger>
              <TabsTrigger value="neutral">Neutral</TabsTrigger>
            </TabsList>

            {/* All Voices Tab */}
            <TabsContent value="all">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {voices.map((voice, index) => (
                  <motion.div
                    key={voice.id}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.05 }}
                  >
                    <Card
                      className={`glass cursor-pointer border-2 transition-all duration-300 hover:border-primary/50 hover:border-primary ${
                        selectedVoice === voice.id
                          ? 'border-primary bg-primary/10'
                          : 'border-border/50 hover:border-border'
                      }`}
                      onClick={() => handleVoiceSelect(voice.id)}
                    >
                      <CardContent className="p-6">
                        <div className="flex items-start gap-4">
                          <div className={`w-14 h-14 rounded-full flex items-center justify-center flex-shrink-0 ${
                            selectedVoice === voice.id
                              ? 'bg-primary'
                              : 'bg-muted'
                          }`}>
                            <Mic className="w-6 h-6 text-white" />
                          </div>

                          <div className="flex-1">
                            <div className="flex items-start justify-between mb-2">
                              <div>
                                <h3 className="text-xl font-bold">{voice.displayName}</h3>
                                <div className="flex gap-2 mt-1">
                                  <Badge variant="secondary" className="capitalize">
                                    {voice.gender}
                                  </Badge>
                                  <Badge variant="outline" className="capitalize">
                                    {voice.age}
                                  </Badge>
                                </div>
                              </div>
                              {selectedVoice === voice.id && (
                                <CheckCircle2 className="w-5 h-5 text-primary" />
                              )}
                            </div>
                            <div className="flex gap-2 text-xs text-muted-foreground">
                              <span>• {voice.accent}</span>
                              <span>• {voice.style}</span>
                            </div>
                          </div>

                          <p className="text-sm text-muted-foreground mb-4">
                            Perfect for {voice.style.toLowerCase()} {voice.contentType?.toLowerCase() || 'educational'} content
                          </p>

                          <div className="flex items-center gap-4">
                            {/* Audio Player */}
                            <div className="flex-1 flex items-center gap-3">
                              <Button
                                variant="ghost"
                                size="sm"
                                onClick={() => setIsPlaying(!isPlaying)}
                              >
                                {isPlaying ? <Pause className="w-4 h-4" /> : <Play className="w-4 h-4" />}
                              </Button>
                              <div className="flex-1">
                                <div className="h-2 bg-primary/20 rounded-full overflow-hidden">
                                  <motion.div
                                    className="h-full bg-primary rounded-full"
                                    initial={{ width: '0%' }}
                                    animate={{ width: isPlaying ? '100%' : '0%' }}
                                    transition={{ duration: 0.3 }}
                                  />
                                </div>
                              </div>
                              <span className="text-xs text-muted-foreground">0:05</span>
                            </div>
                          </div>

                          <Button className="w-full mt-2" variant={selectedVoice === voice.id ? 'default' : 'outline'}>
                            {selectedVoice === voice.id ? 'Selected' : 'Select Voice'}
                            {selectedVoice === voice.id && <ChevronRight className="w-4 h-4 ml-2" />}
                          </Button>
                        </div>
                      </CardContent>
                    </Card>
                  </motion.div>
                ))}
              </div>
            </TabsContent>

            {/* Male Voices Tab */}
            <TabsContent value="male">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {voices.filter(v => v.gender === 'male').map((voice, index) => (
                  <motion.div
                    key={voice.id}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.05 }}
                  >
                    <Card
                      className={`glass cursor-pointer border-2 transition-all duration-300 hover:border-primary/50 ${
                        selectedVoice === voice.id
                          ? 'border-primary bg-primary/10'
                          : 'border-border/50'
                      }`}
                      onClick={() => handleVoiceSelect(voice.id)}
                    >
                      <CardContent className="p-6">
                        <div className="flex items-start gap-4">
                          <div className={`w-14 h-14 rounded-full flex items-center justify-center flex-shrink-0 ${
                            selectedVoice === voice.id
                              ? 'bg-primary'
                              : 'bg-muted'
                          }`}>
                            <Mic className="w-6 h-6 text-white" />
                          </div>

                          <div className="flex-1">
                            <h3 className="text-xl font-bold mb-2">{voice.displayName}</h3>
                            <div className="flex gap-2 text-xs text-muted-foreground mb-4">
                              <span>• {voice.accent}</span>
                              <span>• {voice.style}</span>
                              <span>• {voice.age}</span>
                            </div>

                            <Button className="w-full" variant={selectedVoice === voice.id ? 'default' : 'outline'}>
                              {selectedVoice === voice.id ? 'Selected' : 'Select Voice'}
                            </Button>
                          </div>
                        </div>
                      </CardContent>
                    </Card>
                  </motion.div>
                ))}
              </div>
            </TabsContent>

            {/* Female Voices Tab */}
            <TabsContent value="female">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {voices.filter(v => v.gender === 'female').map((voice, index) => (
                  <motion.div
                    key={voice.id}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.05 }}
                  >
                    <Card
                      className={`glass cursor-pointer border-2 transition-all duration-300 hover:border-primary/50 ${
                        selectedVoice === voice.id
                          ? 'border-primary bg-primary/10'
                          : 'border-border/50'
                      }`}
                      onClick={() => handleVoiceSelect(voice.id)}
                    >
                      <CardContent className="p-6">
                        <div className="flex items-start gap-4">
                          <div className={`w-14 h-14 rounded-full flex items-center justify-center flex-shrink-0 ${
                            selectedVoice === voice.id
                              ? 'bg-primary'
                              : 'bg-pink-500'
                          }`}>
                            <Mic className="w-6 h-6 text-white" />
                          </div>

                          <div className="flex-1">
                            <h3 className="text-xl font-bold mb-2">{voice.displayName}</h3>
                            <div className="flex gap-2 text-xs text-muted-foreground mb-4">
                              <span>• {voice.accent}</span>
                              <span>• {voice.style}</span>
                              <span>• {voice.age}</span>
                            </div>

                            <Button className="w-full" variant={selectedVoice === voice.id ? 'default' : 'outline'}>
                              {selectedVoice === voice.id ? 'Selected' : 'Select Voice'}
                            </Button>
                          </div>
                        </div>
                      </CardContent>
                    </Card>
                  </motion.div>
                ))}
              </div>
            </TabsContent>

            {/* Neutral Voices Tab */}
            <TabsContent value="neutral">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {voices.filter(v => v.gender === 'neutral').map((voice, index) => (
                  <motion.div
                    key={voice.id}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.05 }}
                  >
                    <Card
                      className={`glass cursor-pointer border-2 transition-all duration-300 hover:border-primary/50 ${
                        selectedVoice === voice.id
                          ? 'border-purple-500 bg-purple-500/10'
                          : 'border-border/50'
                      }`}
                      onClick={() => handleVoiceSelect(voice.id)}
                    >
                      <CardContent className="p-6">
                        <div className="flex items-start gap-4">
                          <div className={`w-14 h-14 rounded-full flex items-center justify-center flex-shrink-0 ${
                            selectedVoice === voice.id
                              ? 'bg-purple-500'
                              : 'bg-muted'
                          }`}>
                            <Mic className="w-6 h-6 text-white" />
                          </div>

                          <div className="flex-1">
                            <h3 className="text-xl font-bold mb-2">{voice.displayName}</h3>
                            <div className="flex gap-2 text-xs text-muted-foreground mb-4">
                              <span>• {voice.accent}</span>
                              <span>• {voice.style}</span>
                            </div>

                            <Button className="w-full" variant={selectedVoice === voice.id ? 'default' : 'outline'}>
                              {selectedVoice === voice.id ? 'Selected' : 'Select Voice'}
                            </Button>
                          </div>
                        </div>
                      </CardContent>
                    </Card>
                  </motion.div>
                ))}
              </div>
            </TabsContent>
          </Tabs>

          {/* Volume Control */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.2 }}
            className="mt-8"
          >
            <Card className="glass">
              <CardContent className="p-6">
                <div className="flex items-center gap-4 mb-4">
                  <Volume2 className="w-6 h-6 text-primary" />
                  <div className="flex-1">
                    <h3 className="font-semibold mb-2">Voice Volume</h3>
                    <p className="text-sm text-muted-foreground">Adjust the volume for all AI-generated voices</p>
                  </div>
                  <Badge variant="outline">{volume}%</Badge>
                </div>
                <Slider
                  value={[volume]}
                  onValueChange={([value]) => setVolume(value)}
                  max={100}
                  step={1}
                  className="flex-1"
                />
                <Button variant="outline" size="sm" onClick={() => setVolume(100)}>
                  <RefreshCw className="w-4 h-4 mr-2" />
                  Reset
                </Button>
              </CardContent>
            </Card>
          </motion.div>

          {/* Selected Voice Info */}
          {selectedVoice && (
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.3 }}
              className="mt-6"
            >
              <Card className="glass border-primary/50">
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <CheckCircle2 className="w-5 h-5 text-primary" />
                      <CardTitle>Voice Selected</CardTitle>
                    </div>
                    <Button variant="ghost" size="sm">
                      <Info className="w-4 h-4" />
                    </Button>
                  </div>
                  <CardDescription>
                    {voices.find(v => v.id === selectedVoice)?.displayName} is now set as your default voice for AI video generation
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="grid md:grid-cols-3 gap-6">
                    <div className="space-y-2">
                      <div className="flex items-center gap-2 text-sm text-muted-foreground">
                        <Users className="w-4 h-4" />
                        <span>Perfect for {voices.find(v => v.id === selectedVoice)?.age} students</span>
                      </div>
                    </div>
                    <div className="space-y-2">
                      <div className="flex items-center gap-2 text-sm text-muted-foreground">
                        <Clock className="w-4 h-4" />
                        <span>Fast generation speed</span>
                      </div>
                    </div>
                    <div className="space-y-2">
                      <div className="flex items-center gap-2 text-sm text-muted-foreground">
                        <Star className="w-4 h-4 text-yellow-500" />
                        <span>High quality TTS</span>
                      </div>
                    </div>
                  </div>

                  <div className="flex gap-4 mt-6">
                    <Link href="/ai-video/generate" className="flex-1">
                      <Button size="lg" className="gradient-border">
                        Continue to Video Generation
                        <ChevronRight className="w-5 h-5 ml-2" />
                      </Button>
                    </Link>
                    <Link href="/student">
                      <Button size="lg" variant="outline">
                        Back to Dashboard
                      </Button>
                    </Link>
                  </div>
                </CardContent>
              </Card>
            </motion.div>
          )}
        </div>
      </main>
    </div>
  )
}
