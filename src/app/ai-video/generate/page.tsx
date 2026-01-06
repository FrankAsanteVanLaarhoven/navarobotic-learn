'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Input } from '@/components/ui/input'
import { Textarea } from '@/components/ui/textarea'
import { Slider } from '@/components/ui/slider'
import { Label } from '@/components/ui/label'
import { Progress } from '@/components/ui/progress'
import { Avatar, AvatarFallback } from '@/components/ui/avatar'
import {
  Video,
  Play,
  Pause,
  RefreshCw,
  Settings,
  Sparkles,
  Wand2,
  FileText,
  Clock,
  CheckCircle2,
  Download,
  ChevronRight,
  Mic,
  Globe,
  Users,
  Zap
} from 'lucide-react'
import { useState } from 'react'
import Link from 'next/link'

export default function VideoGenerationPage() {
  const [activeTab, setActiveTab] = useState('generate')
  const [selectedCourse, setSelectedCourse] = useState<string | null>(null)
  const [selectedModel, setSelectedModel] = useState<string>('sora')
  const [selectedVoice, setSelectedVoice] = useState<string>('voice-adam')
  const [prompt, setPrompt] = useState('')
  const [duration, setDuration] = useState(30)
  const [resolution, setResolution] = useState('1080p')
  const [isGenerating, setIsGenerating] = useState(false)
  const [generationProgress, setGenerationProgress] = useState(0)

  const videoModels = [
    { id: 'sora', name: 'Sora', provider: 'OpenAI', maxDuration: 60, resolution: '1080p', cost: 0.20, description: 'Industry-leading text-to-video with cinematic quality', icon: '🎬' },
    { id: 'kling', name: 'Kling AI', provider: 'Kling', maxDuration: 120, resolution: '1080p', cost: 0.10, description: 'High-quality video generation with realistic movements', icon: '✨' },
    { id: 'veo', name: 'Veo', provider: 'Veo', maxDuration: 60, resolution: '4K', cost: 0.05, description: 'Cinematic AI video at affordable prices', icon: '🎥' },
    { id: 'synthara', name: 'Synthara', provider: 'Synthara', maxDuration: 180, resolution: '4K', cost: 0.08, description: 'Photorealistic AI video generation', icon: '🎞' },
    { id: 'runway', name: 'Runway', provider: 'RunwayML', maxDuration: 90, resolution: '1080p', cost: 0.25, description: 'Professional video for creators', icon: '🎓' },
    { id: 'luma', name: 'Luma', provider: 'Luma', maxDuration: 60, resolution: '720p', cost: 0.15, description: 'Realistic 3D video generation', icon: '🎨' },
    { id: 'pika', name: 'Pika', provider: 'Pika', maxDuration: 120, resolution: '1080p', cost: 0.18, description: 'Fast AI video generation', icon: '🎬' },
  ]

  const courses = [
    { id: 'course-electronics', slug: 'electronics-humanoid-robots', title: 'Electronics of Humanoid Robots', modules: 8, lessons: 32, icon: <FileText className="w-5 h-5" /> },
    { id: 'course-python', slug: 'python-robotics', title: 'Python for Robotics', modules: 6, lessons: 24, icon: <FileText className="w-5 h-5" /> },
    { id: 'course-control', slug: 'unitree-g1-fundamentals', title: 'Unitree G1 Fundamentals', modules: 4, lessons: 16, icon: <FileText className="w-5 h-5" /> },
    { id: 'course-ros2', slug: 'ros2-ai-integration', title: 'ROS2 & AI Integration', modules: 5, lessons: 20, icon: <FileText className="w-5 h-5" /> },
  ]

  const handleGenerate = async () => {
    if (!selectedCourse || !prompt.trim()) {
      alert('Please select a course and enter a prompt')
      return
    }

    setIsGenerating(true)
    setGenerationProgress(0)

    // Simulate video generation progress
    const progressInterval = setInterval(() => {
      setGenerationProgress(prev => {
        if (prev >= 100) {
          clearInterval(progressInterval)
          setIsGenerating(false)
          return 100
        }
        return prev + Math.random() * 10
      })
    }, 500)

    // In production, this would call your backend API
    console.log('Generating video:', {
      courseId: selectedCourse,
      modelId: selectedModel,
      voiceId: selectedVoice,
      prompt,
      duration,
      resolution
    })
  }

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-4">
              <Link href="/student" className="flex items-center gap-2">
                <Video className="h-8 w-8 text-primary" />
                <span className="text-lg font-bold gradient-text">AI VIDEO STUDIO</span>
              </Link>
              <Badge variant="default" className="glow">
                <Sparkles className="w-3 h-3 mr-1" />
                AI Powered
              </Badge>
            </div>

            <div className="flex items-center gap-4">
              <Link href="/ai-video/voices" className="text-sm text-muted-foreground hover:text-foreground transition-colors">
                <Mic className="w-4 h-4 mr-1" />
                Voices
              </Link>
              <Button variant="ghost" size="sm">
                <Settings className="w-4 h-4" />
              </Button>
              <div className="flex items-center gap-2 pl-4 border-l border-border">
                <Avatar className="w-8 h-8">
                  <AvatarFallback>US</AvatarFallback>
                </Avatar>
              </div>
            </div>
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="pt-20 pb-8 px-4">
        <div className="container mx-auto max-w-7xl">
          <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
            <TabsList className="grid w-full max-w-md grid-cols-3 mb-8">
              <TabsTrigger value="generate">
                <Wand2 className="w-4 h-4 mr-2" />
                Generate
              </TabsTrigger>
              <TabsTrigger value="library">
                <FileText className="w-4 h-4 mr-2" />
                Library
              </TabsTrigger>
              <TabsTrigger value="settings">
                <Settings className="w-4 h-4 mr-2" />
                Settings
              </TabsTrigger>
            </TabsList>

            {/* Generate Tab */}
            <TabsContent value="generate">
              <div className="grid lg:grid-cols-3 gap-6">
                {/* Left: Course & Model Selection */}
                <div className="lg:col-span-1 space-y-6">
                  {/* Course Selection */}
                  <Card className="glass">
                    <CardHeader>
                      <CardTitle>Select Course</CardTitle>
                      <CardDescription>Choose a course to generate video for</CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ScrollArea className="h-64">
                        <div className="space-y-2">
                          {courses.map((course) => (
                            <button
                              key={course.id}
                              onClick={() => setSelectedCourse(course.id)}
                              className={`w-full text-left p-4 rounded-lg transition-all duration-200 ${
                                selectedCourse === course.id
                                  ? 'bg-primary border-primary'
                                  : 'bg-muted/30 hover:bg-muted/50 border-transparent'
                              }`}
                            >
                              <div className="flex items-center gap-3">
                                <div className={`w-10 h-10 rounded-full flex items-center justify-center flex-shrink-0 ${
                                  selectedCourse === course.id ? 'bg-white text-primary' : 'bg-primary/20'
                                }`}>
                                  {course.icon}
                                </div>
                                <div className="flex-1 min-w-0">
                                  <h4 className={`font-semibold text-sm mb-1 ${
                                    selectedCourse === course.id ? 'text-white' : 'text-foreground'
                                  }`}>
                                    {course.title}
                                  </h4>
                                  <div className={`flex gap-3 text-xs ${
                                    selectedCourse === course.id ? 'text-white/80' : 'text-muted-foreground'
                                  }`}>
                                    <span>{course.modules} modules</span>
                                    <span>•</span>
                                    <span>{course.lessons} lessons</span>
                                  </div>
                                </div>
                              </div>
                            </button>
                          ))}
                        </div>
                      </ScrollArea>
                    </CardContent>
                  </Card>

                  {/* Video Model Selection */}
                  <Card className="glass">
                    <CardHeader>
                      <CardTitle>Video Model</CardTitle>
                      <CardDescription>Choose AI video generation model</CardDescription>
                    </CardHeader>
                    <CardContent>
                      <ScrollArea className="h-80">
                        <div className="space-y-3">
                          {videoModels.map((model) => (
                            <button
                              key={model.id}
                              onClick={() => setSelectedModel(model.id)}
                              className={`w-full text-left p-4 rounded-lg border-2 transition-all duration-200 ${
                                selectedModel === model.id
                                  ? 'bg-primary/10 border-primary'
                                  : 'bg-muted/30 hover:bg-muted/50 border-transparent'
                              }`}
                            >
                              <div className="flex items-start gap-3">
                                <div className="text-2xl">{model.icon}</div>
                                <div className="flex-1">
                                  <h4 className={`font-semibold text-sm mb-1 ${
                                    selectedModel === model.id ? 'text-primary' : 'text-foreground'
                                  }`}>
                                    {model.name}
                                  </h4>
                                  <div className="flex gap-2 text-xs">
                                    <Badge variant="outline">{model.provider}</Badge>
                                    <Badge variant="secondary">${model.cost}/min</Badge>
                                  </div>
                                  <p className={`text-xs text-muted-foreground ${
                                    selectedModel === model.id ? 'text-primary' : ''
                                  }`}>
                                    {model.description}
                                  </p>
                                  <div className="flex gap-2 text-xs mt-2">
                                    <span className="flex items-center gap-1">
                                      <Clock className="w-3 h-3" />
                                      Max: {model.maxDuration}s
                                    </span>
                                    <span className="flex items-center gap-1">
                                      <Video className="w-3 h-3" />
                                      {model.resolution}
                                    </span>
                                  </div>
                                </div>
                              </div>
                            </button>
                          ))}
                        </div>
                      </ScrollArea>
                    </CardContent>
                  </Card>

                  {/* Voice Selection */}
                  <Card className="glass">
                    <CardHeader>
                      <div className="flex items-center justify-between">
                        <div>
                          <CardTitle>Voice</CardTitle>
                          <CardDescription>Choose AI voice for narration</CardDescription>
                        </div>
                        <Link href="/ai-video/voices" className="text-sm text-primary hover:underline">
                          View All Voices
                        </Link>
                      </div>
                    </CardHeader>
                    <CardContent className="space-y-2">
                      {['voice-adam', 'voice-bella'].map((voice) => (
                        <button
                          key={voice}
                          onClick={() => setSelectedVoice(voice)}
                          className={`w-full text-left p-3 rounded-lg border-2 transition-all duration-200 flex items-center gap-3 ${
                            selectedVoice === voice
                              ? 'bg-primary/10 border-primary'
                              : 'bg-muted/30 hover:bg-muted/50 border-transparent'
                          }`}
                        >
                          <Mic className={`w-5 h-5 ${
                            selectedVoice === voice ? 'text-primary' : 'text-muted-foreground'
                          }`} />
                          <div>
                            <h4 className={`font-semibold text-sm ${
                              selectedVoice === voice ? 'text-primary' : 'text-foreground'
                            }`}>
                              {voice === 'voice-adam' ? 'Adam (Male)' : 'Bella (Female)'}
                            </h4>
                            <p className="text-xs text-muted-foreground">ElevenLabs TTS</p>
                          </div>
                        </button>
                      ))}
                      <Link href="/ai-video/voices" className="w-full mt-2">
                        <Button variant="outline" size="sm" className="w-full">
                          <Mic className="w-4 h-4 mr-2" />
                          Browse All Voices
                        </Button>
                      </Link>
                    </CardContent>
                  </Card>
                </div>

                {/* Right: Prompt & Generation */}
                <div className="lg:col-span-2 space-y-6">
                  {/* Prompt Input */}
                  <Card className="glass">
                    <CardHeader>
                      <CardTitle>Video Prompt</CardTitle>
                      <CardDescription>Describe the video you want to generate</CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-4">
                      <div className="space-y-2">
                        <Label htmlFor="prompt">Prompt</Label>
                        <Textarea
                          id="prompt"
                          placeholder="Describe the robot, its actions, and environment..."
                          value={prompt}
                          onChange={(e) => setPrompt(e.target.value)}
                          rows={6}
                          className="resize-none"
                        />
                      </div>

                      {/* Quick Prompt Templates */}
                      <div className="grid grid-cols-2 gap-2">
                        <Button
                          variant="outline"
                          size="sm"
                          onClick={() => setPrompt('A humanoid robot walking naturally in a modern laboratory setting, showing its gait control mechanisms')}
                        >
                          Walking Demo
                        </Button>
                        <Button
                          variant="outline"
                          size="sm"
                          onClick={() => setPrompt('A Unitree G1 robot performing yoga-like stretching exercises, demonstrating flexibility and range of motion')}
                        >
                          Yoga Demo
                        </Button>
                      </div>

                      {/* Course-Based Prompts */}
                      {selectedCourse && (
                        <div className="mt-4 p-4 bg-primary/10 rounded-lg">
                          <p className="text-sm font-medium text-primary mb-2">
                            <FileText className="w-4 h-4 inline mr-1" />
                            Course-Based Prompt
                          </p>
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => setPrompt('Generate an engaging educational video teaching the core concepts of ' + courses.find(c => c.id === selectedCourse)?.title + ' with clear explanations and visual demonstrations')}
                            className="w-full"
                          >
                            Generate Lesson Video
                          </Button>
                        </div>
                      )}
                    </CardContent>
                  </Card>

                  {/* Video Settings */}
                  <Card className="glass">
                    <CardHeader>
                      <CardTitle>Video Settings</CardTitle>
                      <CardDescription>Configure video quality and duration</CardDescription>
                    </CardHeader>
                    <CardContent className="space-y-6">
                      <div className="space-y-2">
                        <Label>Duration: {duration}s</Label>
                        <Slider
                          value={[duration]}
                          onValueChange={([value]) => setDuration(value)}
                          max={videoModels.find(m => m.id === selectedModel)?.maxDuration || 180}
                          step={1}
                          className="flex-1"
                        />
                        <Button
                          variant="ghost"
                          size="sm"
                          onClick={() => setDuration(30)}
                        >
                          <RefreshCw className="w-4 h-4" />
                        </Button>
                      </div>

                      <div className="space-y-2">
                        <Label>Resolution: {resolution}</Label>
                        <div className="flex gap-2">
                          {['720p', '1080p', '4K'].map((res) => (
                            <button
                              key={res}
                              onClick={() => setResolution(res)}
                              className={`flex-1 px-3 py-2 rounded-lg border-2 transition-all duration-200 ${
                                resolution === res
                                  ? 'bg-primary border-primary text-white'
                                  : 'bg-muted/30 hover:bg-muted/50 border-transparent'
                              }`}
                            >
                              {res}
                            </button>
                          ))}
                        </div>
                      </div>

                      <div className="flex items-center justify-between p-3 bg-muted/30 rounded-lg">
                        <div className="flex items-center gap-2">
                          <Globe className="w-4 h-4 text-primary" />
                          <span className="text-sm">
                            Estimated cost: <span className="font-bold">${(duration * (videoModels.find(m => m.id === selectedModel)?.cost || 0)).toFixed(2)}</span>
                          </span>
                        </div>
                        <Badge variant="outline">
                          <Zap className="w-3 h-3 mr-1" />
                          Fast Generation
                        </Badge>
                      </div>
                    </CardContent>
                  </Card>

                  {/* Generate Button */}
                  <Card className={`glass border-2 ${isGenerating ? 'border-primary animate-pulse' : 'border-border/50'}`}>
                    <CardContent className="p-6">
                      {isGenerating ? (
                        <div className="space-y-4">
                          <div className="flex items-center justify-center">
                            <div className="relative w-20 h-20">
                              <motion.div
                                className="absolute inset-0 bg-primary/20 rounded-full"
                                animate={{ rotate: 360 }}
                                transition={{ duration: 1, repeat: Infinity, ease: 'linear' }}
                              />
                              <motion.div
                                className="absolute inset-2 bg-primary rounded-full"
                                animate={{
                                  scale: [1, 1.5, 1],
                                  rotate: [0, 90, 180, 270],
                                  borderRadius: ['20%', '20%', '50%', '50%']
                                }}
                                transition={{ duration: 2, repeat: Infinity }}
                              />
                            </div>
                          </div>
                          <h3 className="text-center font-bold text-primary">Generating Your Video...</h3>
                          <p className="text-center text-sm text-muted-foreground">
                            This may take a few minutes depending on length
                          </p>
                          <div className="space-y-2">
                            <div className="flex justify-between text-sm">
                              <span className="text-muted-foreground">Progress</span>
                              <span className="font-bold text-primary">{generationProgress.toFixed(0)}%</span>
                            </div>
                            <Progress value={generationProgress} className="h-2" />
                          </div>
                        </div>
                      ) : (
                        <div className="space-y-4">
                          <div className="text-center">
                            <Wand2 className="w-16 h-16 text-primary mx-auto mb-4" />
                            <h3 className="text-xl font-bold mb-2">Generate Video</h3>
                            <p className="text-sm text-muted-foreground">
                              Create an AI-generated video for your course using {videoModels.find(m => m.id === selectedModel)?.name}
                            </p>
                          </div>

                          <Button
                            onClick={handleGenerate}
                            disabled={!selectedCourse || !prompt.trim()}
                            size="lg"
                            className="w-full gradient-border"
                          >
                            <Sparkles className="w-5 h-5 mr-2" />
                            Start Generation
                          </Button>

                          {!selectedCourse && (
                            <p className="text-xs text-center text-muted-foreground mt-2">
                              Please select a course to continue
                            </p>
                          )}
                          {!prompt.trim() && (
                            <p className="text-xs text-center text-muted-foreground mt-2">
                              Please enter a prompt to continue
                            </p>
                          )}
                        </div>
                      )}
                    </CardContent>
                  </Card>
                </div>
              </div>
            </TabsContent>

            {/* Library Tab */}
            <TabsContent value="library">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                <Card className="glass lg:col-span-3">
                  <CardHeader>
                    <CardTitle>My Generated Videos</CardTitle>
                    <CardDescription>View and download your AI-generated course videos</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="text-center py-12">
                      <FileText className="w-16 h-16 text-muted-foreground mx-auto mb-4" />
                      <h3 className="text-xl font-bold mb-2">No Videos Generated Yet</h3>
                      <p className="text-muted-foreground max-w-md mx-auto">
                        Start generating videos for your courses to see them here. Generated videos will be stored in your library for offline viewing.
                      </p>
                      <Button
                        onClick={() => setActiveTab('generate')}
                        className="mt-4"
                      >
                        <Wand2 className="w-5 h-5 mr-2" />
                        Generate Your First Video
                      </Button>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            {/* Settings Tab */}
            <TabsContent value="settings">
              <div className="grid md:grid-cols-2 gap-6">
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Generation Defaults</CardTitle>
                    <CardDescription>Default settings for new video generations</CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="space-y-2">
                      <Label>Default Model</Label>
                      <select className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm">
                        <option value="sora">Sora</option>
                        <option value="kling">Kling AI</option>
                        <option value="veo">Veo</option>
                        <option value="synthara">Synthara</option>
                      </select>
                    </div>
                    <div className="space-y-2">
                      <Label>Default Voice</Label>
                      <select className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm">
                        <option value="voice-adam">Adam (Male)</option>
                        <option value="voice-bella">Bella (Female)</option>
                      </select>
                    </div>
                    <div className="space-y-2">
                      <Label>Default Duration</Label>
                      <div className="flex gap-2">
                        {[15, 30, 45, 60].map((dur) => (
                          <button
                            key={dur}
                            className="flex-1 px-3 py-2 rounded-lg border-2 bg-muted/30 hover:bg-muted/50"
                          >
                            {dur}s
                          </button>
                        ))}
                      </div>
                    </div>
                    <Button className="w-full">
                      <Settings className="w-4 h-4 mr-2" />
                      Save Defaults
                    </Button>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Usage Statistics</CardTitle>
                    <CardDescription>Your video generation usage and costs</CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="flex items-center justify-between p-3 bg-muted/30 rounded-lg">
                      <div>
                        <p className="text-sm text-muted-foreground">Videos Generated</p>
                        <p className="text-2xl font-bold">0</p>
                      </div>
                      <Video className="w-8 h-8 text-primary" />
                    </div>
                    <div className="flex items-center justify-between p-3 bg-muted/30 rounded-lg">
                      <div>
                        <p className="text-sm text-muted-foreground">Total Duration</p>
                        <p className="text-2xl font-bold">0m</p>
                      </div>
                      <Clock className="w-8 h-8 text-primary" />
                    </div>
                    <div className="flex items-center justify-between p-3 bg-muted/30 rounded-lg">
                      <div>
                        <p className="text-sm text-muted-foreground">Total Cost</p>
                        <p className="text-2xl font-bold text-green-500">$0.00</p>
                      </div>
                      <Zap className="w-8 h-8 text-yellow-500" />
                    </div>
                    <div className="p-3 bg-primary/10 rounded-lg">
                      <p className="text-sm text-center">
                        <Users className="w-5 h-5 mx-auto mb-2 text-primary" />
                        <span className="font-bold">Unlimited Credits</span>
                      </p>
                      <p className="text-xs text-center text-muted-foreground">
                        Subscribe to Pro for more video generations
                      </p>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>
          </Tabs>
        </div>
      </main>
    </div>
  )
}
