'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Input } from '@/components/ui/input'
import { Avatar, AvatarFallback, AvatarImage } from '@/components/ui/avatar'
import {
  Video,
  VideoOff,
  Mic,
  MicOff,
  MonitorSpeaker,
  MessageSquare,
  Phone,
  PhoneOff,
  Share,
  Clock,
  Users,
  Calendar,
  Plus,
  Settings,
  ChevronRight,
  Download,
  Play,
  MoreHorizontal,
  Send,
  Bell,
  Circle
} from 'lucide-react'
import { useState } from 'react'
import Link from 'next/link'

export default function VideoPortal() {
  const [activeTab, setActiveTab] = useState('sessions')
  const [isMuted, setIsMuted] = useState(false)
  const [isVideoOff, setIsVideoOff] = useState(false)
  const [isScreenShare, setIsScreenShare] = useState(false)

  const liveSessions = [
    { id: 1, title: 'Control Systems: Live Lab', course: 'Control Systems', instructor: 'Dr. Smith', viewers: 45, startTime: '14:00', status: 'live' },
    { id: 2, title: 'Python Workshop Session', course: 'Python for Robotics', instructor: 'Prof. Johnson', viewers: 128, startTime: '10:30', status: 'live' },
  ]

  const upcomingSessions = [
    { id: 3, title: 'Robotics Workshop: Day 3', course: 'Unitree G1 Fundamentals', instructor: 'Dr. Williams', startTime: '15:00', date: 'Tomorrow', registered: 45 },
    { id: 4, title: 'Advanced Control Q&A', course: 'Control Systems', instructor: 'Dr. Brown', startTime: '11:00', date: 'Tomorrow', registered: 23 },
  ]

  const recordings = [
    { id: 1, title: 'Introduction to Unitree G1', duration: '1:45:32', views: 1243, date: '2 days ago', thumbnail: '/images/unitree-g1.png' },
    { id: 2, title: 'Python Functions Deep Dive', duration: '2:30:15', views: 856, date: '1 week ago', thumbnail: '/images/ros2-diagram.png' },
    { id: 3, title: 'Control Systems Lab Session', duration: '3:15:22', views: 2341, date: '2 weeks ago', thumbnail: '/images/simulation-interface.png' },
    { id: 4, title: 'Q&A Session: Advanced Topics', duration: '1:20:45', views: 567, date: '3 weeks ago', thumbnail: '/images/learning-dashboard.png' },
  ]

  const chatMessages = [
    { user: 'John Doe', message: 'Can you explain the PID control parameters?', time: '14:05', avatar: 'JD' },
    { user: 'Dr. Smith', message: 'Sure! The K, I, and D terms...', time: '14:06', isInstructor: true },
    { user: 'Sarah Johnson', message: 'Thanks for the clarification!', time: '14:08', avatar: 'SJ' },
  ]

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-4">
              <Link href="/auth" className="flex items-center gap-2">
                <Video className="h-8 w-8 text-primary" />
                <span className="text-lg font-bold gradient-text">VIDEO PORTAL</span>
              </Link>
              <Badge variant="default">LIVE</Badge>
            </div>

            <div className="flex items-center gap-4">
              <Button variant="ghost" size="sm">
                <MessageSquare className="w-4 h-4" />
              </Button>
              <div className="flex items-center gap-2 pl-4 border-l border-border">
                <Avatar className="w-8 h-8">
                  <AvatarImage src="/images/unitree-g1.png" />
                  <AvatarFallback>US</AvatarFallback>
                </Avatar>
                <div className="text-right">
                  <p className="text-sm font-medium">User</p>
                  <p className="text-xs text-muted-foreground">View Profile</p>
                </div>
              </div>
              <Button variant="ghost" size="sm">
                <Settings className="w-4 h-4" />
              </Button>
            </div>
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="pt-20 pb-8 px-4">
        <div className="container mx-auto">
          <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
            <TabsList className="grid w-full max-w-2xl grid-cols-4 mb-6">
              <TabsTrigger value="sessions">
                <Video className="w-4 h-4 mr-2" />
                Live Sessions
              </TabsTrigger>
              <TabsTrigger value="upcoming">
                <Calendar className="w-4 h-4 mr-2" />
                Upcoming
              </TabsTrigger>
              <TabsTrigger value="recordings">
                <Circle className="w-4 h-4 mr-2" />
                Recordings
              </TabsTrigger>
              <TabsTrigger value="my-rooms">
                <Plus className="w-4 h-4 mr-2" />
                My Rooms
              </TabsTrigger>
            </TabsList>

            {/* Live Sessions Tab */}
            <TabsContent value="sessions">
              <div className="grid lg:grid-cols-3 gap-6">
                {/* Current Live Session */}
                <div className="lg:col-span-2">
                  <Card className="glass border-primary/50">
                    <CardHeader className="bg-primary/10">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2">
                          <Badge className="animate-ping">LIVE NOW</Badge>
                          <CardTitle>Control Systems: Live Lab</CardTitle>
                        </div>
                        <Button variant="destructive" size="sm">
                          <PhoneOff className="w-4 h-4 mr-2" />
                          End Session
                        </Button>
                      </div>
                      <CardDescription>
                        <div className="flex items-center gap-4">
                          <div className="flex items-center gap-2">
                            <Users className="w-4 h-4" />
                            <span className="font-semibold">45</span> viewers
                          </div>
                          <div className="flex items-center gap-2">
                            <Clock className="w-4 h-4" />
                            <span className="font-semibold">Session Time: 45:32</span>
                          </div>
                        </div>
                        Instructor: <span className="font-semibold">Dr. Smith</span>
                      </CardDescription>
                    </CardHeader>
                    <CardContent>
                      {/* Video Feed */}
                      <div className="aspect-video bg-black rounded-lg mb-4 flex items-center justify-center relative overflow-hidden">
                        <div className="absolute inset-0 flex items-center justify-center">
                          <div className="text-center">
                            <Video className="w-16 h-16 text-muted-foreground/50 mx-auto mb-2" />
                            <p className="text-muted-foreground">Video Feed</p>
                            <p className="text-sm text-muted-foreground/70">Connecting to session...</p>
                          </div>
                        </div>

                        {/* Video Controls */}
                        <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-black/90 to-transparent p-4">
                          <div className="flex items-center justify-between">
                            <div className="flex items-center gap-2">
                              <Button variant="ghost" size="sm">
                                <Mic className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm">
                                <Video className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm">
                                <MonitorSpeaker className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm" onClick={() => setIsMuted(!isMuted)}>
                                {isMuted ? (
                                  <MicOff className="w-4 h-4" />
                                ) : (
                                  <Mic className="w-4 h-4" />
                                )}
                              </Button>
                            </div>

                            <div className="flex items-center gap-2">
                              <Button variant="ghost" size="sm" onClick={() => setIsScreenShare(!isScreenShare)}>
                                <Share className="w-4 h-4 mr-2" />
                                {isScreenShare ? 'Sharing' : 'Share Screen'}
                              </Button>
                              <Button variant="ghost" size="sm">
                                <Circle className="w-4 h-4 mr-2" />
                                Record
                              </Button>
                            </div>
                          </div>
                        </div>
                      </div>

                      {/* Chat Panel */}
                      <div className="mt-4">
                        <div className="flex items-center justify-between mb-3">
                          <h4 className="font-semibold">Session Chat</h4>
                          <span className="text-sm text-muted-foreground">
                            <MessageSquare className="w-4 h-4 inline mr-1" />
                            24 messages
                          </span>
                        </div>
                        <ScrollArea className="h-48 border rounded-lg bg-muted/30 p-3 mb-3">
                          <div className="space-y-3">
                            {chatMessages.map((msg, index) => (
                              <div key={index} className={`flex gap-3 ${msg.isInstructor ? 'bg-primary/10 rounded-lg p-2' : ''}`}>
                                <Avatar className="w-8 h-8 flex-shrink-0">
                                  <AvatarFallback>{msg.avatar}</AvatarFallback>
                                </Avatar>
                                <div className="flex-1 min-w-0">
                                  <div className="flex items-center gap-2 mb-1">
                                    <span className="font-medium text-sm">{msg.user}</span>
                                    <span className="text-xs text-muted-foreground">{msg.time}</span>
                                  </div>
                                  <p className="text-sm">{msg.message}</p>
                                </div>
                              </div>
                            ))}
                          </div>
                        </ScrollArea>
                        <div className="flex gap-2">
                          <Input placeholder="Type a message..." className="flex-1" />
                          <Button>
                            <Send className="w-4 h-4 mr-2" />
                            Send
                          </Button>
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                </div>

                {/* Other Live Sessions */}
                <div className="space-y-4">
                  <h3 className="text-lg font-semibold mb-3">Other Live Sessions</h3>
                  {liveSessions.slice(1).map((session) => (
                    <Card key={session.id} className="glass hover:border-primary/50 transition-all cursor-pointer">
                      <CardContent className="p-4">
                        <div className="flex items-center gap-3 mb-3">
                          <Badge className="animate-ping">LIVE</Badge>
                          <div className="flex-1">
                            <h4 className="font-semibold">{session.title}</h4>
                            <p className="text-sm text-muted-foreground">{session.course}</p>
                          </div>
                        </div>
                        <div className="flex items-center gap-2 mb-3">
                          <Users className="w-4 h-4 text-primary" />
                          <span className="text-sm">{session.viewers} watching</span>
                        </div>
                        <p className="text-xs text-muted-foreground mb-2">
                          Started at {session.startTime} • {session.instructor}
                        </p>
                        <Button className="w-full" size="sm">
                          <Video className="w-4 h-4 mr-2" />
                          Join Session
                        </Button>
                      </CardContent>
                    </Card>
                  ))}
                </div>
              </div>
            </TabsContent>

            {/* Upcoming Tab */}
            <TabsContent value="upcoming">
              <div className="space-y-4">
                <h2 className="text-2xl font-bold mb-6">Upcoming Sessions</h2>
                {upcomingSessions.map((session) => (
                  <motion.div
                    key={session.id}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: session.id * 0.1 }}
                  >
                    <Card className="glass">
                      <CardContent className="p-6">
                        <div className="flex items-start gap-4">
                          <div className="w-12 h-12 rounded-lg bg-accent/20 flex items-center justify-center flex-shrink-0">
                            <Calendar className="w-6 h-6 text-accent" />
                          </div>
                          <div className="flex-1">
                            <h3 className="text-lg font-semibold mb-2">{session.title}</h3>
                            <p className="text-sm text-muted-foreground mb-3">{session.course}</p>
                            <div className="flex items-center gap-6 text-sm mb-3">
                              <span className="flex items-center gap-2">
                                <Clock className="w-4 h-4" />
                                {session.date} at {session.startTime}
                              </span>
                              <span className="flex items-center gap-2">
                                <Users className="w-4 h-4" />
                                {session.registered} registered
                              </span>
                            </div>
                            <div className="flex items-center gap-2">
                              <span className="font-medium">Instructor:</span>
                              <span>{session.instructor}</span>
                            </div>
                          </div>
                          <div className="flex gap-2">
                            <Button className="flex-1" variant="outline">
                              <Calendar className="w-4 h-4 mr-2" />
                              Add to Calendar
                            </Button>
                            <Button>
                              <Bell className="w-4 h-4 mr-2" />
                              Set Reminder
                            </Button>
                          </div>
                        </div>
                      </CardContent>
                    </Card>
                  </motion.div>
                ))}
              </div>
            </TabsContent>

            {/* Recordings Tab */}
            <TabsContent value="recordings">
              <div className="space-y-4">
                <h2 className="text-2xl font-bold mb-6">Session Recordings</h2>
                <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                  {recordings.map((recording) => (
                    <motion.div
                      key={recording.id}
                      initial={{ opacity: 0, y: 20 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ delay: recording.id * 0.1 }}
                    >
                      <Card className="glass hover:border-primary/50 transition-all cursor-pointer group">
                        <div className="relative h-40 overflow-hidden">
                          <img
                            src={recording.thumbnail}
                            alt={recording.title}
                            className="w-full h-full object-cover group-hover:scale-110 transition-transform duration-300"
                          />
                          <div className="absolute inset-0 bg-gradient-to-t from-background via-background/60 to-transparent" />
                          <div className="absolute bottom-0 left-0 right-0 p-3 flex items-center justify-between">
                            <Badge className="text-xs">{recording.duration}</Badge>
                            <div className="flex items-center gap-2">
                              <span className="text-sm flex items-center gap-1">
                                <Play className="w-3 h-3" />
                                {recording.views.toLocaleString()} views
                              </span>
                              <Clock className="w-3 h-3 text-muted-foreground" />
                            </div>
                          </div>
                        </div>
                        <CardContent className="p-4">
                          <h3 className="font-semibold mb-2 line-clamp-1">{recording.title}</h3>
                          <p className="text-xs text-muted-foreground mb-4">{recording.date}</p>
                          <div className="flex gap-2">
                            <Button className="flex-1" size="sm">
                              <Play className="w-4 h-4 mr-2" />
                              Watch
                            </Button>
                            <Button variant="outline" size="sm">
                              <Download className="w-4 h-4 mr-2" />
                            </Button>
                          </div>
                        </CardContent>
                      </Card>
                    </motion.div>
                  ))}
                </div>
              </div>
            </TabsContent>

            {/* My Rooms Tab */}
            <TabsContent value="my-rooms">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {/* Create New Room Button */}
                <motion.div
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                >
                  <Card className="glass border-dashed border-2 hover:border-primary/50 transition-all cursor-pointer h-full min-h-64 flex flex-col items-center justify-center">
                    <div className="text-center">
                      <div className="w-16 h-16 rounded-full bg-primary/20 flex items-center justify-center mx-auto mb-4">
                        <Plus className="w-8 h-8 text-primary" />
                      </div>
                      <h3 className="text-lg font-semibold mb-2">Create New Room</h3>
                      <p className="text-sm text-muted-foreground">Start a new video session</p>
                    </div>
                  </Card>
                </motion.div>

                {/* Existing Rooms */}
                {[1, 2, 3, 4, 5].map((room) => (
                  <motion.div
                    key={room}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: room * 0.1 }}
                  >
                    <Card className="glass hover:border-primary/50 transition-all cursor-pointer group">
                      <CardContent className="p-6">
                        <div className="w-full aspect-video bg-gradient-to-br from-muted/20 to-muted/30 rounded-lg mb-4 flex items-center justify-center group-hover:from-primary/10 group-hover:to-primary/5 transition-all">
                          <Video className="w-12 h-12 text-muted-foreground/50" />
                          <Badge className="absolute top-4 right-4">Room {room}</Badge>
                        </div>
                        <div className="space-y-2">
                          <div className="flex items-center justify-between">
                            <h3 className="font-semibold">Room {room}</h3>
                            <Badge variant={room % 2 === 0 ? 'default' : 'secondary'}>
                              {room % 2 === 0 ? 'Active' : 'Inactive'}
                            </Badge>
                          </div>
                          <p className="text-sm text-muted-foreground">Session Room</p>
                          <p className="text-xs text-muted-foreground">Last used: Today</p>
                          <div className="flex gap-2 mt-3">
                            <Button className="flex-1" size="sm">
                              <Video className="w-4 h-4 mr-2" />
                              Enter Room
                            </Button>
                            <Button variant="outline" size="sm">
                              <Settings className="w-4 h-4" />
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
        </div>
      </main>
    </div>
  )
}
