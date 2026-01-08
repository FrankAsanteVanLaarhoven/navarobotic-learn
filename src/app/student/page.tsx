'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Progress } from '@/components/ui/progress'
import { Separator } from '@/components/ui/separator'
import {
  BookOpen,
  Clock,
  Play,
  Video,
  Calendar,
  Trophy,
  Target,
  Award,
  MessageSquare,
  Bell,
  Settings,
  LogOut,
  User,
  TrendingUp,
  CheckCircle2,
  ChevronRight,
  Plus,
  MonitorPlay,
  Wand2,
  Mic,
  FileText
} from 'lucide-react'
import Link from 'next/link'
import { useState } from 'react'
import { AnimatedLogo } from '@/components/AnimatedLogo'

export default function StudentPortal() {
  const [activeTab, setActiveTab] = useState('dashboard')

  const recentCourses = [
    { title: 'Unitree G1 Fundamentals', progress: 75, nextLesson: 'Walking Control Basics', image: '/images/unitree-g1.png' },
    { title: 'Python for Robotics', progress: 45, nextLesson: 'Functions & Classes', image: '/images/ros2-diagram.png' },
    { title: 'Control Systems', progress: 20, nextLesson: 'PID Control', image: '/images/simulation-interface.png' },
  ]

  const upcomingSessions = [
    { title: 'Live Q&A Session', course: 'Control Systems', time: '14:00 Today', instructor: 'Dr. Smith', status: 'live' },
    { title: 'Robotics Workshop', course: 'Unitree G1', time: '10:00 Tomorrow', instructor: 'Prof. Johnson', status: 'upcoming' },
    { title: 'Lab Session', course: 'Python for Robotics', time: '15:00 Tomorrow', instructor: 'Dr. Williams', status: 'upcoming' },
  ]

  const achievements = [
    { title: 'First Course Completed', date: '2 days ago', icon: <Award className="w-5 h-5" /> },
    { title: '100 Lessons Watched', date: '1 week ago', icon: <Trophy className="w-5 h-5" /> },
    { title: '7 Day Streak', date: '2 weeks ago', icon: <Target className="w-5 h-5" /> },
  ]

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-4">
              <Link href="/" className="flex items-center gap-2">
                <AnimatedLogo size="md" />
                <span className="text-lg font-bold gradient-text">STUDENT PORTAL</span>
              </Link>
            </div>

            <div className="flex items-center gap-4">
              <Button variant="ghost" size="sm">
                <Bell className="w-4 h-4" />
              </Button>
              <Button variant="ghost" size="sm">
                <MessageSquare className="w-4 h-4" />
              </Button>
              <div className="flex items-center gap-2 pl-4 border-l border-border">
                <div className="w-8 h-8 rounded-full bg-primary/20 flex items-center justify-center">
                  <User className="w-4 h-4 text-primary" />
                </div>
                <div className="text-right">
                  <p className="text-sm font-medium">John Doe</p>
                  <p className="text-xs text-muted-foreground">Student</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="pt-20 pb-8 px-4">
        <div className="container mx-auto">
          {/* Welcome Section */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            className="mb-8"
          >
            <h1 className="text-3xl font-bold mb-2">
              Welcome back, <span className="gradient-text">John</span>! 👋
            </h1>
            <p className="text-muted-foreground">
              You have 3 courses in progress. Keep up the great work!
            </p>
          </motion.div>

          <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
            <TabsList className="grid w-full max-w-2xl grid-cols-5 mb-6">
              <TabsTrigger value="dashboard">
                <BookOpen className="w-4 h-4 mr-2" />
                Dashboard
              </TabsTrigger>
              <TabsTrigger value="courses">
                <BookOpen className="w-4 h-4 mr-2" />
                My Courses
              </TabsTrigger>
              <TabsTrigger value="schedule">
                <Calendar className="w-4 h-4 mr-2" />
                Schedule
              </TabsTrigger>
              <TabsTrigger value="achievements">
                <Trophy className="w-4 h-4 mr-2" />
                Achievements
              </TabsTrigger>
              <TabsTrigger value="ai-video">
                <Video className="w-4 h-4 mr-2" />
                AI Video Studio
              </TabsTrigger>
            </TabsList>

            {/* Dashboard Tab */}
            <TabsContent value="dashboard">
              <div className="grid lg:grid-cols-3 gap-6">
                {/* Stats */}
                <div className="lg:col-span-2 grid md:grid-cols-2 gap-4 mb-6">
                  <Card className="glass">
                    <CardContent className="p-6">
                      <div className="flex items-center justify-between">
                        <div>
                          <p className="text-sm text-muted-foreground">Courses Enrolled</p>
                          <p className="text-3xl font-bold">12</p>
                        </div>
                        <div className="w-12 h-12 rounded-full bg-primary/20 flex items-center justify-center">
                          <BookOpen className="w-6 h-6 text-primary" />
                        </div>
                      </div>
                    </CardContent>
                  </Card>

                  <Card className="glass">
                    <CardContent className="p-6">
                      <div className="flex items-center justify-between">
                        <div>
                          <p className="text-sm text-muted-foreground">Hours Learned</p>
                          <p className="text-3xl font-bold">48h</p>
                        </div>
                        <div className="w-12 h-12 rounded-full bg-accent/20 flex items-center justify-center">
                          <Clock className="w-6 h-6 text-accent" />
                        </div>
                      </div>
                    </CardContent>
                  </Card>

                  <Card className="glass">
                    <CardContent className="p-6">
                      <div className="flex items-center justify-between">
                        <div>
                          <p className="text-sm text-muted-foreground">Lessons Completed</p>
                          <p className="text-3xl font-bold">89</p>
                        </div>
                        <div className="w-12 h-12 rounded-full bg-green-500/20 flex items-center justify-center">
                          <CheckCircle2 className="w-6 h-6 text-green-500" />
                        </div>
                      </div>
                    </CardContent>
                  </Card>

                  <Card className="glass">
                    <CardContent className="p-6">
                      <div className="flex items-center justify-between">
                        <div>
                          <p className="text-sm text-muted-foreground">Certificates</p>
                          <p className="text-3xl font-bold">3</p>
                        </div>
                        <div className="w-12 h-12 rounded-full bg-purple-500/20 flex items-center justify-center">
                          <Award className="w-6 h-6 text-purple-500" />
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                </div>

                {/* Progress Overview */}
                <Card className="glass h-full">
                  <CardHeader>
                    <CardTitle>Weekly Progress</CardTitle>
                    <CardDescription>Your learning activity</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      {['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'].map((day, i) => (
                        <div key={day} className="flex items-center gap-3">
                          <span className="text-xs text-muted-foreground w-8">{day}</span>
                          <Progress value={[30, 45, 60, 80, 40, 70, 90][i]} className="flex-1" />
                          <span className="text-xs font-semibold w-10">{
                            [30, 45, 60, 80, 40, 70, 90][i]}%</span>
                        </div>
                      ))}
                    </div>
                  </CardContent>
                </Card>
              </div>

              {/* Recent Courses */}
              <div className="mb-6">
                <h2 className="text-2xl font-bold mb-4 flex items-center gap-2">
                  <TrendingUp className="w-6 h-6 text-primary" />
                  Continue Learning
                </h2>
                <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                  {recentCourses.map((course, index) => (
                    <motion.div
                      key={index}
                      initial={{ opacity: 0, y: 20 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ delay: index * 0.1 }}
                    >
                      <Card className="glass hover:border-primary/50 transition-all duration-300 cursor-pointer">
                        <div className="relative h-40 overflow-hidden">
                          <img
                            src={course.image}
                            alt={course.title}
                            className="w-full h-full object-cover"
                          />
                          <div className="absolute inset-0 bg-gradient-to-t from-background to-transparent" />
                          <Badge className="absolute top-4 right-4 bg-primary">In Progress</Badge>
                        </div>
                        <CardContent className="p-4">
                          <h3 className="font-semibold mb-2">{course.title}</h3>
                          <div className="space-y-2">
                            <div>
                              <div className="flex justify-between text-sm mb-1">
                                <span className="text-muted-foreground">Progress</span>
                                <span className="font-semibold">{course.progress}%</span>
                              </div>
                              <Progress value={course.progress} />
                            </div>
                            <p className="text-sm text-muted-foreground">
                              Next: {course.nextLesson}
                            </p>
                            <Button className="w-full mt-3" size="sm">
                              <Play className="w-4 h-4 mr-2" />
                              Continue
                            </Button>
                          </div>
                        </CardContent>
                      </Card>
                    </motion.div>
                  ))}
                </div>
              </div>

              {/* Upcoming Sessions */}
              <Card className="glass">
                <CardHeader>
                  <CardTitle className="flex items-center gap-2">
                    <Video className="w-5 h-5 text-primary" />
                    Upcoming Sessions
                  </CardTitle>
                  <CardDescription>Live classes and meetings</CardDescription>
                </CardHeader>
                <CardContent>
                  <ScrollArea className="h-64">
                    <div className="space-y-3">
                      {upcomingSessions.map((session, index) => (
                        <div key={index} className="p-4 rounded-lg bg-muted/30 hover:bg-muted/50 transition-colors">
                          <div className="flex items-start justify-between">
                            <div className="flex-1">
                              <div className="flex items-center gap-2 mb-2">
                                <Badge variant={session.status === 'live' ? 'default' : 'secondary'}>
                                  {session.status === 'live' ? (
                                    <>
                                      <span className="relative flex h-2 w-2 mr-1">
                                        <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-red-400 opacity-75"></span>
                                        <span className="relative inline-flex rounded-full h-2 w-2 bg-red-500"></span>
                                      </span>
                                      Live Now
                                    </>
                                  ) : 'Upcoming'}
                                </Badge>
                                <h4 className="font-semibold">{session.title}</h4>
                              </div>
                              <p className="text-sm text-muted-foreground">{session.course}</p>
                              <div className="flex items-center gap-4 text-sm mt-2">
                                <span className="flex items-center gap-1">
                                  <Calendar className="w-4 h-4" />
                                  {session.time}
                                </span>
                                <span className="flex items-center gap-1">
                                  <User className="w-4 h-4" />
                                  {session.instructor}
                                </span>
                              </div>
                            </div>
                            <Link href={`/video/session/${session.title}`}>
                              <Button size="sm" variant={session.status === 'live' ? 'default' : 'outline'}>
                                <MonitorPlay className="w-4 h-4 mr-2" />
                                Join
                              </Button>
                            </Link>
                          </div>
                        </div>
                      ))}
                    </div>
                  </ScrollArea>
                </CardContent>
              </Card>

              {/* Achievements Preview */}
              <Card className="glass">
                <CardHeader>
                  <CardTitle className="flex items-center gap-2">
                    <Award className="w-5 h-5 text-primary" />
                    Recent Achievements
                  </CardTitle>
                  <CardDescription>Your latest accomplishments</CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="space-y-3">
                    {achievements.map((achievement, index) => (
                      <div key={index} className="flex items-center gap-3 p-3 rounded-lg bg-muted/30">
                        <div className="w-10 h-10 rounded-full bg-primary/20 flex items-center justify-center flex-shrink-0">
                          <div className="text-primary">{achievement.icon}</div>
                        </div>
                        <div className="flex-1">
                          <p className="font-semibold text-sm">{achievement.title}</p>
                          <p className="text-xs text-muted-foreground">{achievement.date}</p>
                        </div>
                      </div>
                    ))}
                  </div>
                  <Link href="/student?tab=achievements">
                    <Button variant="outline" className="w-full mt-4">
                      View All Achievements
                      <ChevronRight className="w-4 h-4 ml-2" />
                    </Button>
                  </Link>
                </CardContent>
              </Card>
            </TabsContent>

            {/* Courses Tab */}
            <TabsContent value="courses">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {[...recentCourses, ...recentCourses].map((course, index) => (
                  <Card key={index} className="glass hover:border-primary/50 transition-all duration-300 cursor-pointer">
                    <div className="relative h-48 overflow-hidden">
                      <img
                        src={course.image}
                        alt={course.title}
                        className="w-full h-full object-cover"
                      />
                      <div className="absolute inset-0 bg-gradient-to-t from-background to-transparent" />
                      <Badge className="absolute top-4 right-4">In Progress</Badge>
                    </div>
                    <CardContent className="p-4">
                      <h3 className="font-semibold mb-2">{course.title}</h3>
                      <div>
                        <div className="flex justify-between text-sm mb-1">
                          <span className="text-muted-foreground">Progress</span>
                          <span className="font-semibold">{course.progress}%</span>
                        </div>
                        <Progress value={course.progress} />
                      </div>
                      <Button className="w-full mt-3" size="sm">
                        <Play className="w-4 h-4 mr-2" />
                        Continue
                      </Button>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </TabsContent>

            {/* Schedule Tab */}
            <TabsContent value="schedule">
              <Card className="glass">
                <CardHeader>
                  <CardTitle>Weekly Schedule</CardTitle>
                  <CardDescription>Your upcoming classes and sessions</CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="space-y-4">
                    {['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday'].map((day) => (
                      <div key={day} className="border-b border-border/50 pb-4 last:border-0">
                        <h3 className="font-semibold mb-3">{day}</h3>
                        <div className="space-y-2">
                          <div className="flex items-center gap-3 p-3 rounded-lg bg-primary/10 hover:bg-primary/20 transition-colors cursor-pointer">
                            <div className="w-2 h-2 rounded-full bg-red-500" />
                            <div className="flex-1">
                              <p className="font-medium text-sm">Control Systems: Live Lab</p>
                              <p className="text-xs text-muted-foreground">10:00 AM - 11:30 AM</p>
                            </div>
                            <Button size="sm" variant="outline">Join</Button>
                          </div>
                          <div className="flex items-center gap-3 p-3 rounded-lg bg-accent/10 hover:bg-accent/20 transition-colors cursor-pointer">
                            <div className="w-2 h-2 rounded-full bg-blue-500" />
                            <div className="flex-1">
                              <p className="font-medium text-sm">Python Workshop</p>
                              <p className="text-xs text-muted-foreground">2:00 PM - 3:30 PM</p>
                            </div>
                            <Button size="sm" variant="outline">Join</Button>
                          </div>
                        </div>
                      </div>
                    ))}
                  </div>
                </CardContent>
              </Card>
            </TabsContent>

            {/* Achievements Tab */}
            <TabsContent value="achievements">
              <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                {[...achievements, ...achievements, ...achievements].map((achievement, index) => (
                  <Card key={index} className="glass">
                    <CardContent className="p-6 text-center">
                      <div className="w-16 h-16 rounded-full bg-gradient-to-br from-primary/20 to-accent/20 flex items-center justify-center mx-auto mb-4">
                        <div className="text-primary text-2xl">{achievement.icon}</div>
                      </div>
                      <h3 className="font-semibold mb-2">{achievement.title}</h3>
                      <p className="text-sm text-muted-foreground">{achievement.date}</p>
                      <Badge className="mt-2" variant="secondary">Unlocked</Badge>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </TabsContent>

            {/* AI Video Studio Tab */}
            <TabsContent value="ai-video">
              <div className="text-center py-16">
                <div className="max-w-md mx-auto">
                  <Card className="glass">
                    <CardContent className="p-8">
                      <Video className="w-16 h-16 text-primary mx-auto mb-4" />
                      <h2 className="text-2xl font-bold mb-4">AI Video Studio</h2>
                      <p className="text-muted-foreground mb-6">
                        Generate personalized video walkthroughs for your courses using advanced AI models and natural-sounding voices.
                      </p>
                      <Link href="/ai-video/generate" className="w-full">
                        <Button size="lg" className="gradient-border">
                          <Wand2 className="w-5 h-5 mr-2" />
                          Create AI Video
                        </Button>
                      </Link>
                      <div className="grid grid-cols-2 gap-4 mt-4">
                        <Link href="/ai-video/voices" className="w-full">
                          <Button variant="outline" className="w-full">
                            <Mic className="w-4 h-4 mr-2" />
                            Browse Voices
                          </Button>
                        </Link>
                        <Link href="/ai-video/generate" className="w-full">
                          <Button variant="outline" className="w-full">
                            <FileText className="w-4 h-4 mr-2" />
                            View Library
                          </Button>
                        </Link>
                      </div>
                      <div className="mt-6 pt-6 border-t border-border/50">
                        <p className="text-xs text-muted-foreground text-center">
                          Powered by Sora, Kling, Veo, Synthara, Runway, Luma, Pika and ElevenLabs voices
                        </p>
                      </div>
                    </CardContent>
                  </Card>
                </div>
              </div>
            </TabsContent>
          </Tabs>
        </div>
      </main>
    </div>
  )
}
