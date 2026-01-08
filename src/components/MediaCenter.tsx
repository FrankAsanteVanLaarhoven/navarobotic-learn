'use client'

import { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
  DropdownMenuSeparator,
} from '@/components/ui/dropdown-menu'
import { ScrollArea } from '@/components/ui/scroll-area'
import {
  Play,
  Pause,
  Volume2,
  VolumeX,
  Maximize,
  Minimize,
  Settings,
  Download,
  Share2,
  PictureInPicture,
  SkipForward,
  SkipBack,
  Repeat,
  Shuffle,
  MoreVertical,
  PlayCircle,
  Clock,
  Newspaper,
  ExternalLink,
  RefreshCw,
} from 'lucide-react'
import { MissionImpossibleText } from '@/components/MissionImpossibleText'

interface Video {
  id: string
  title: string
  url: string
  thumbnail?: string
  duration?: string
}

const videos: Video[] = [
  {
    id: '1',
    title: 'Robotics Tutorial 1',
    url: 'https://www.youtube.com/embed/sTjxOtlG4-c',
    duration: '10:30',
  },
  {
    id: '2',
    title: 'Robotics Tutorial 2',
    url: 'https://www.youtube.com/embed/Eu5mYMavctM',
    duration: '15:45',
  },
  {
    id: '3',
    title: 'Robotics Tutorial 3',
    url: 'https://www.youtube.com/embed/zEYIcaQwn6s?start=41',
    duration: '12:20',
  },
  {
    id: '4',
    title: 'Robotics Tutorial 4',
    url: 'https://www.youtube.com/embed/_ZlX1Jz-TEQ',
    duration: '18:15',
  },
  {
    id: '5',
    title: 'Robotics Tutorial 5',
    url: 'https://www.youtube.com/embed/LVPr2nfzcbk',
    duration: '9:50',
  },
  {
    id: '6',
    title: 'Robotics Tutorial 6',
    url: 'https://www.youtube.com/embed/jx1DHC8JLT4',
    duration: '14:30',
  },
  {
    id: '7',
    title: 'Robotics Tutorial 7',
    url: 'https://www.youtube.com/embed/n1Mi3ISXNjc',
    duration: '11:25',
  },
  {
    id: '8',
    title: 'Robotics Tutorial 8',
    url: 'https://www.youtube.com/embed/59KR11Lyhas',
    duration: '13:45',
  },
]

interface NewsItem {
  id: string
  title: string
  description: string
  url: string
  source: string
  publishedAt: string
  image?: string
}

// Mock robotics news feed - In production, this would fetch from RSS/API
const fetchRoboticsNews = async (): Promise<NewsItem[]> => {
  // Simulate API call delay
  await new Promise(resolve => setTimeout(resolve, 500))
  
  return [
    {
      id: '1',
      title: 'Latest Breakthrough in Humanoid Robotics',
      description: 'Researchers achieve new milestones in humanoid robot locomotion and manipulation capabilities.',
      url: 'https://example.com/news/1',
      source: 'Robotics Today',
      publishedAt: new Date(Date.now() - 2 * 60 * 60 * 1000).toISOString(),
    },
    {
      id: '2',
      title: 'AI-Powered Robot Learns Complex Tasks in Minutes',
      description: 'New reinforcement learning algorithm enables robots to master new skills rapidly.',
      url: 'https://example.com/news/2',
      source: 'Tech Robotics',
      publishedAt: new Date(Date.now() - 5 * 60 * 60 * 1000).toISOString(),
    },
    {
      id: '3',
      title: 'Industrial Robotics Market Reaches $50B',
      description: 'The global industrial robotics market continues to grow with automation adoption.',
      url: 'https://example.com/news/3',
      source: 'Industry Report',
      publishedAt: new Date(Date.now() - 8 * 60 * 60 * 1000).toISOString(),
    },
    {
      id: '4',
      title: 'Open Source ROS 2 Framework Updates',
      description: 'Latest ROS 2 release brings improved performance and new middleware options.',
      url: 'https://example.com/news/4',
      source: 'Open Source Robotics',
      publishedAt: new Date(Date.now() - 12 * 60 * 60 * 1000).toISOString(),
    },
    {
      id: '5',
      title: 'Medical Robotics: Surgical Precision Advances',
      description: 'New surgical robots offer enhanced precision for minimally invasive procedures.',
      url: 'https://example.com/news/5',
      source: 'Medical Tech',
      publishedAt: new Date(Date.now() - 24 * 60 * 60 * 1000).toISOString(),
    },
  ]
}

export function MediaCenter() {
  const [selectedVideo, setSelectedVideo] = useState<Video>(videos[0])
  const [isPlaying, setIsPlaying] = useState(false)
  const [isMuted, setIsMuted] = useState(false)
  const [isFullscreen, setIsFullscreen] = useState(false)
  const [volume, setVolume] = useState(100)
  const [repeatMode, setRepeatMode] = useState(false)
  const [shuffleMode, setShuffleMode] = useState(false)
  const [newsItems, setNewsItems] = useState<NewsItem[]>([])
  const [isLoadingNews, setIsLoadingNews] = useState(false)
  const [activeTab, setActiveTab] = useState('videos')
  const [videoRef, setVideoRef] = useState<HTMLIFrameElement | null>(null)

  const extractVideoId = (url: string) => {
    const match = url.match(/(?:youtube\.com\/embed\/|youtu\.be\/|youtube\.com\/watch\?v=)([^&\n?#]+)/)
    return match ? match[1] : null
  }

  const getThumbnailUrl = (url: string) => {
    const videoId = extractVideoId(url)
    return videoId ? `https://img.youtube.com/vi/${videoId}/maxresdefault.jpg` : ''
  }

  // Load news feed on mount and when tab changes
  useEffect(() => {
    if (activeTab === 'news' && newsItems.length === 0) {
      loadNews()
    }
  }, [activeTab])

  // Listen for fullscreen changes
  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement)
    }

    document.addEventListener('fullscreenchange', handleFullscreenChange)
    return () => document.removeEventListener('fullscreenchange', handleFullscreenChange)
  }, [])

  const loadNews = async () => {
    setIsLoadingNews(true)
    try {
      const news = await fetchRoboticsNews()
      setNewsItems(news)
    } catch (error) {
      console.error('Failed to load news:', error)
    } finally {
      setIsLoadingNews(false)
    }
  }

  const handleFullscreen = () => {
    if (!isFullscreen) {
      const player = document.querySelector('.video-player-container')
      if (player && player.requestFullscreen) {
        player.requestFullscreen()
      }
    } else {
      if (document.exitFullscreen) {
        document.exitFullscreen()
      }
    }
    setIsFullscreen(!isFullscreen)
  }

  const handleShare = async () => {
    if (navigator.share) {
      try {
        await navigator.share({
          title: selectedVideo.title,
          text: `Watch: ${selectedVideo.title}`,
          url: selectedVideo.url.replace('/embed/', '/watch?v='),
        })
      } catch (error) {
        console.error('Error sharing:', error)
      }
    } else {
      // Fallback: copy to clipboard
      navigator.clipboard.writeText(selectedVideo.url.replace('/embed/', '/watch?v='))
      alert('Link copied to clipboard!')
    }
  }

  const handleDownload = () => {
    // In production, this would trigger a download or open download options
    window.open(selectedVideo.url.replace('/embed/', '/watch?v='), '_blank')
  }

  const handlePictureInPicture = async () => {
    const iframe = document.querySelector('iframe')
    if (iframe && (iframe as any).requestPictureInPicture) {
      try {
        await (iframe as any).requestPictureInPicture()
      } catch (error) {
        console.error('Picture-in-Picture not supported:', error)
      }
    }
  }

  const handleSkipBack = () => {
    const iframe = document.querySelector('iframe') as HTMLIFrameElement
    if (iframe && iframe.contentWindow) {
      iframe.contentWindow.postMessage('{"event":"command","func":"seekTo","args":[0,true]}', '*')
    }
  }

  const handleSkipForward = () => {
    const iframe = document.querySelector('iframe') as HTMLIFrameElement
    if (iframe && iframe.contentWindow) {
      // Skip forward 10 seconds
      iframe.contentWindow.postMessage('{"event":"command","func":"seekBy","args":[10,true]}', '*')
    }
  }

  const handlePlayPause = () => {
    const iframe = document.querySelector('iframe') as HTMLIFrameElement
    if (iframe && iframe.contentWindow) {
      if (isPlaying) {
        iframe.contentWindow.postMessage('{"event":"command","func":"pauseVideo","args":""}', '*')
      } else {
        iframe.contentWindow.postMessage('{"event":"command","func":"playVideo","args":""}', '*')
      }
      setIsPlaying(!isPlaying)
    }
  }

  const formatTimeAgo = (dateString: string) => {
    const date = new Date(dateString)
    const now = new Date()
    const diffInHours = Math.floor((now.getTime() - date.getTime()) / (1000 * 60 * 60))
    
    if (diffInHours < 1) return 'Just now'
    if (diffInHours < 24) return `${diffInHours}h ago`
    const diffInDays = Math.floor(diffInHours / 24)
    return `${diffInDays}d ago`
  }

  return (
    <section className="py-20 px-4">
      <div className="container mx-auto">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
        >
          <div className="text-center mb-12">
            <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-4">
              Media Center
            </MissionImpossibleText>
            <MissionImpossibleText variant="body" glowIntensity="medium" className="max-w-2xl mx-auto">
              Watch and learn from our comprehensive robotics video library
            </MissionImpossibleText>
          </div>

          <div className="grid lg:grid-cols-3 gap-6">
            {/* Main Video Player */}
            <div className="lg:col-span-2">
              <Card className="glass border-border/50 overflow-hidden">
                <CardHeader className="pb-3">
                  <div className="flex items-center justify-between">
                    <CardTitle className="flex items-center gap-2">
                      <PlayCircle className="w-5 h-5 text-primary" />
                      <MissionImpossibleText variant="label" glowIntensity="medium">
                        {selectedVideo.title}
                      </MissionImpossibleText>
                    </CardTitle>
                    <DropdownMenu>
                      <DropdownMenuTrigger asChild>
                        <Button variant="ghost" size="sm" className="h-8 w-8 p-0">
                          <MoreVertical className="w-4 h-4" />
                        </Button>
                      </DropdownMenuTrigger>
                      <DropdownMenuContent align="end" className="glass border-border/50 w-56">
                        <DropdownMenuItem 
                          className="flex items-center gap-2 cursor-pointer"
                          onClick={handleDownload}
                        >
                          <Download className="w-4 h-4" />
                          Download
                        </DropdownMenuItem>
                        <DropdownMenuItem 
                          className="flex items-center gap-2 cursor-pointer"
                          onClick={handleShare}
                        >
                          <Share2 className="w-4 h-4" />
                          Share
                        </DropdownMenuItem>
                        <DropdownMenuItem 
                          className="flex items-center gap-2 cursor-pointer"
                          onClick={handlePictureInPicture}
                        >
                          <PictureInPicture className="w-4 h-4" />
                          Picture in Picture
                        </DropdownMenuItem>
                        <DropdownMenuSeparator />
                        <DropdownMenuItem 
                          className="flex items-center gap-2 cursor-pointer"
                          onClick={() => {
                            // Open media controls menu
                            document.querySelector('[data-media-controls-trigger]')?.click()
                          }}
                        >
                          <Settings className="w-4 h-4" />
                          Media Controls
                        </DropdownMenuItem>
                      </DropdownMenuContent>
                    </DropdownMenu>
                  </div>
                </CardHeader>
                <CardContent className="p-0">
                  <div className="relative aspect-video bg-black rounded-lg overflow-hidden video-player-container">
                    <iframe
                      ref={setVideoRef}
                      src={`${selectedVideo.url}?autoplay=0&controls=1&modestbranding=1&rel=0&enablejsapi=1`}
                      className="w-full h-full"
                      allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                      allowFullScreen
                      title={selectedVideo.title}
                      id="youtube-player"
                    />
                  </div>

                  {/* Hidden Media Controls Menu Button */}
                  <div className="p-4 bg-muted/30 border-t border-border/50 flex justify-end">
                    <DropdownMenu>
                      <DropdownMenuTrigger asChild>
                        <Button 
                          variant="ghost" 
                          size="sm" 
                          className="flex items-center gap-2"
                          data-media-controls-trigger
                        >
                          <Settings className="w-4 h-4" />
                          <span className="hidden sm:inline">Media Controls</span>
                        </Button>
                      </DropdownMenuTrigger>
                      <DropdownMenuContent align="end" className="glass border-border/50 w-64">
                        <div className="p-2 space-y-2">
                          <div className="flex items-center justify-between px-2 py-1">
                            <span className="text-sm">Playback</span>
                            <div className="flex items-center gap-1">
                              <Button
                                variant="ghost"
                                size="sm"
                                onClick={handlePlayPause}
                                className="h-8 w-8 p-0"
                              >
                                {isPlaying ? (
                                  <Pause className="w-4 h-4" />
                                ) : (
                                  <Play className="w-4 h-4" />
                                )}
                              </Button>
                              <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-8 w-8 p-0"
                                onClick={handleSkipBack}
                              >
                                <SkipBack className="w-4 h-4" />
                              </Button>
                              <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-8 w-8 p-0"
                                onClick={handleSkipForward}
                              >
                                <SkipForward className="w-4 h-4" />
                              </Button>
                            </div>
                          </div>
                          <div className="flex items-center justify-between px-2 py-1">
                            <span className="text-sm">Volume</span>
                            <div className="flex items-center gap-2">
                              <Button
                                variant="ghost"
                                size="sm"
                                onClick={() => setIsMuted(!isMuted)}
                                className="h-8 w-8 p-0"
                              >
                                {isMuted ? (
                                  <VolumeX className="w-4 h-4" />
                                ) : (
                                  <Volume2 className="w-4 h-4" />
                                )}
                              </Button>
                              <input
                                type="range"
                                min="0"
                                max="100"
                                value={volume}
                                onChange={(e) => setVolume(Number(e.target.value))}
                                className="w-24 h-2 bg-background rounded-lg appearance-none cursor-pointer"
                              />
                              <span className="text-xs w-8 text-right">{volume}%</span>
                            </div>
                          </div>
                          <div className="flex items-center justify-between px-2 py-1">
                            <span className="text-sm">Display</span>
                            <div className="flex items-center gap-1">
                              <Button
                                variant="ghost"
                                size="sm"
                                onClick={handleFullscreen}
                                className="h-8 w-8 p-0"
                              >
                                {isFullscreen ? (
                                  <Minimize className="w-4 h-4" />
                                ) : (
                                  <Maximize className="w-4 h-4" />
                                )}
                              </Button>
                              <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-8 w-8 p-0"
                                onClick={handlePictureInPicture}
                              >
                                <PictureInPicture className="w-4 h-4" />
                              </Button>
                            </div>
                          </div>
                          <div className="flex items-center justify-between px-2 py-1">
                            <span className="text-sm">Playback Mode</span>
                            <div className="flex items-center gap-1">
                              <Button 
                                variant={repeatMode ? "default" : "ghost"} 
                                size="sm" 
                                className="h-8 w-8 p-0"
                                onClick={() => setRepeatMode(!repeatMode)}
                              >
                                <Repeat className="w-4 h-4" />
                              </Button>
                              <Button 
                                variant={shuffleMode ? "default" : "ghost"} 
                                size="sm" 
                                className="h-8 w-8 p-0"
                                onClick={() => setShuffleMode(!shuffleMode)}
                              >
                                <Shuffle className="w-4 h-4" />
                              </Button>
                            </div>
                          </div>
                        </div>
                      </DropdownMenuContent>
                    </DropdownMenu>
                  </div>
                </CardContent>
              </Card>
            </div>

            {/* Video Playlist & News Feed */}
            <div className="lg:col-span-1">
              <Card className="glass border-border/50 h-full">
                <CardHeader>
                  <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
                    <TabsList className="grid w-full grid-cols-2">
                      <TabsTrigger value="videos" className="flex items-center gap-2">
                        <PlayCircle className="w-4 h-4" />
                        <span className="hidden sm:inline">Videos</span>
                      </TabsTrigger>
                      <TabsTrigger value="news" className="flex items-center gap-2">
                        <Newspaper className="w-4 h-4" />
                        <span className="hidden sm:inline">News</span>
                      </TabsTrigger>
                    </TabsList>

                    <TabsContent value="videos" className="mt-4 m-0">
                      <CardTitle className="flex items-center gap-2 mb-4">
                        <PlayCircle className="w-5 h-5 text-primary" />
                        <MissionImpossibleText variant="label" glowIntensity="medium">
                          Video Library
                        </MissionImpossibleText>
                      </CardTitle>
                      <ScrollArea className="h-[600px]">
                        <div className="space-y-2 p-4">
                          {videos.map((video) => (
                        <motion.div
                          key={video.id}
                          whileHover={{ scale: 1.02 }}
                          whileTap={{ scale: 0.98 }}
                        >
                          <button
                            onClick={() => setSelectedVideo(video)}
                            className={`w-full text-left p-3 rounded-lg border transition-all duration-200 ${
                              selectedVideo.id === video.id
                                ? 'border-primary bg-primary/10 shadow-md'
                                : 'border-border/50 hover:border-primary/50 hover:bg-accent/30'
                            }`}
                          >
                            <div className="flex items-start gap-3">
                              <div className="relative flex-shrink-0">
                                <img
                                  src={getThumbnailUrl(video.url)}
                                  alt={video.title}
                                  className="w-24 h-16 object-cover rounded-md"
                                  onError={(e) => {
                                    e.currentTarget.src = `https://img.youtube.com/vi/${extractVideoId(video.url)}/hqdefault.jpg`
                                  }}
                                />
                                <div className="absolute inset-0 flex items-center justify-center bg-black/40 rounded-md">
                                  <PlayCircle className="w-6 h-6 text-white" />
                                </div>
                                {video.duration && (
                                  <div className="absolute bottom-1 right-1 bg-black/80 text-white text-xs px-1 rounded flex items-center gap-1">
                                    <Clock className="w-3 h-3" />
                                    {video.duration}
                                  </div>
                                )}
                              </div>
                              <div className="flex-1 min-w-0">
                                <MissionImpossibleText
                                  variant="small"
                                  glowIntensity={selectedVideo.id === video.id ? 'medium' : 'low'}
                                  className="font-semibold line-clamp-2"
                                >
                                  {video.title}
                                </MissionImpossibleText>
                              </div>
                            </div>
                          </button>
                        </motion.div>
                          ))}
                        </div>
                      </ScrollArea>
                    </TabsContent>

                    <TabsContent value="news" className="mt-4 m-0">
                      <div className="flex items-center justify-between mb-4">
                        <CardTitle className="flex items-center gap-2">
                          <Newspaper className="w-5 h-5 text-primary" />
                          <MissionImpossibleText variant="label" glowIntensity="medium">
                            Robotics News Feed
                          </MissionImpossibleText>
                        </CardTitle>
                        <Button
                          variant="ghost"
                          size="sm"
                          onClick={loadNews}
                          disabled={isLoadingNews}
                          className="h-8 w-8 p-0"
                        >
                          <RefreshCw className={`w-4 h-4 ${isLoadingNews ? 'animate-spin' : ''}`} />
                        </Button>
                      </div>
                      <ScrollArea className="h-[600px]">
                        <div className="space-y-3 p-4">
                          {isLoadingNews ? (
                            <div className="flex items-center justify-center py-8">
                              <RefreshCw className="w-6 h-6 animate-spin text-primary" />
                            </div>
                          ) : newsItems.length === 0 ? (
                            <div className="text-center py-8 text-muted-foreground">
                              <Newspaper className="w-12 h-12 mx-auto mb-2 opacity-50" />
                              <p>No news available</p>
                            </div>
                          ) : (
                            newsItems.map((item) => (
                              <motion.div
                                key={item.id}
                                whileHover={{ scale: 1.02 }}
                                whileTap={{ scale: 0.98 }}
                              >
                                <Card className="border-border/50 hover:border-primary/50 transition-all duration-200 hover:shadow-md cursor-pointer">
                                  <CardContent className="p-4">
                                    <div className="flex items-start gap-3">
                                      <div className="flex-1 min-w-0">
                                        <div className="flex items-center gap-2 mb-2">
                                          <span className="text-xs text-muted-foreground">{item.source}</span>
                                          <span className="text-xs text-muted-foreground">•</span>
                                          <span className="text-xs text-muted-foreground">{formatTimeAgo(item.publishedAt)}</span>
                                        </div>
                                        <MissionImpossibleText
                                          variant="small"
                                          glowIntensity="medium"
                                          className="font-semibold mb-2 line-clamp-2"
                                        >
                                          {item.title}
                                        </MissionImpossibleText>
                                        <p className="text-sm text-muted-foreground line-clamp-2 mb-3">
                                          {item.description}
                                        </p>
                                        <Button
                                          variant="ghost"
                                          size="sm"
                                          className="h-8 text-xs"
                                          onClick={() => window.open(item.url, '_blank')}
                                        >
                                          Read More
                                          <ExternalLink className="w-3 h-3 ml-1" />
                                        </Button>
                                      </div>
                                    </div>
                                  </CardContent>
                                </Card>
                              </motion.div>
                            ))
                          )}
                        </div>
                      </ScrollArea>
                    </TabsContent>
                  </Tabs>
                </CardHeader>
              </Card>
            </div>
          </div>
        </motion.div>
      </div>
    </section>
  )
}
