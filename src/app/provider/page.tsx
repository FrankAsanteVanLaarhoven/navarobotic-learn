'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Progress } from '@/components/ui/progress'
import {
  Users,
  BookOpen,
  Play,
  DollarSign,
  TrendingUp,
  Plus,
  Video,
  MessageSquare,
  Settings,
  Upload,
  FileText,
  Edit,
  Trash2,
  ChevronRight,
  BarChart3,
  Calendar,
  Award
} from 'lucide-react'
import Link from 'next/link'
import { useState } from 'react'

export default function ProviderPortal() {
  const [activeTab, setActiveTab] = useState('dashboard')

  const myCourses = [
    { id: 1, title: 'Unitree G1 Fundamentals', students: 2453, revenue: '$12,265', progress: 75, status: 'Published' },
    { id: 2, title: 'Python for Robotics', students: 7234, revenue: '$36,170', progress: 60, status: 'Published' },
    { id: 3, title: 'Advanced Control Systems', students: 1892, revenue: '$28,380', progress: 30, status: 'Draft' },
  ]

  const upcomingSessions = [
    { id: 1, title: 'Live Q&A Session', course: 'Control Systems', date: 'Today, 2:00 PM', registered: 45 },
    { id: 2, title: 'Workshop: G1 Walking', course: 'Unitree G1', date: 'Tomorrow, 10:00 AM', registered: 128 },
    { id: 3, title: 'Python Lab Session', course: 'Python for Robotics', date: 'Tomorrow, 3:00 PM', registered: 89 },
  ]

  const studentInquiries = [
    { id: 1, student: 'John Doe', course: 'Unitree G1', subject: 'Question about walking control', status: 'Unanswered', time: '2 hours ago' },
    { id: 2, student: 'Sarah Smith', course: 'Python', subject: 'Help with module 5', status: 'Answered', time: '1 day ago' },
    { id: 3, student: 'Mike Johnson', course: 'Control Systems', subject: 'Code debugging issue', status: 'Unanswered', time: '3 hours ago' },
  ]

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-4">
              <Link href="/" className="flex items-center gap-2">
                <Users className="h-8 w-8 text-primary" />
                <span className="text-lg font-bold gradient-text">PROVIDER PORTAL</span>
              </Link>
            </div>

            <div className="flex items-center gap-4">
              <Button variant="ghost" size="sm">
                <Video className="w-4 h-4" />
              </Button>
              <Button variant="ghost" size="sm">
                <MessageSquare className="w-4 h-4" />
                <Badge className="ml-2">7</Badge>
              </Button>
              <Button variant="ghost" size="sm">
                <Calendar className="w-4 h-4" />
              </Button>
              <div className="flex items-center gap-2 pl-4 border-l border-border">
                <div className="w-8 h-8 rounded-full bg-accent/20 flex items-center justify-center">
                  <Users className="w-4 h-4 text-accent" />
                </div>
                <div className="text-right">
                  <p className="text-sm font-medium">Dr. Smith</p>
                  <p className="text-xs text-muted-foreground">Instructor</p>
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
              Welcome back, <span className="gradient-text">Dr. Smith</span>! 👋
            </h1>
            <p className="text-muted-foreground">
              You have 3 active courses with 12,579 enrolled students
            </p>
          </motion.div>

          <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
            <TabsList className="grid w-full max-w-3xl grid-cols-5 mb-6">
              <TabsTrigger value="dashboard">
                <BarChart3 className="w-4 h-4 mr-2" />
                Dashboard
              </TabsTrigger>
              <TabsTrigger value="courses">
                <BookOpen className="w-4 h-4 mr-2" />
                My Courses
              </TabsTrigger>
              <TabsTrigger value="students">
                <Users className="w-4 h-4 mr-2" />
                Students
              </TabsTrigger>
              <TabsTrigger value="schedule">
                <Calendar className="w-4 h-4 mr-2" />
                Schedule
              </TabsTrigger>
              <TabsTrigger value="earnings">
                <DollarSign className="w-4 h-4 mr-2" />
                Earnings
              </TabsTrigger>
            </TabsList>

            {/* Dashboard Tab */}
            <TabsContent value="dashboard">
              <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-4 mb-8">
                <Card className="glass">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="w-12 h-12 rounded-full bg-primary/20 flex items-center justify-center">
                        <Users className="w-6 h-6 text-primary" />
                      </div>
                      <Badge variant="secondary">+12%</Badge>
                    </div>
                    <p className="text-sm text-muted-foreground">Total Students</p>
                    <p className="text-3xl font-bold">12,579</p>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="w-12 h-12 rounded-full bg-green-500/20 flex items-center justify-center">
                        <BookOpen className="w-6 h-6 text-green-500" />
                      </div>
                      <Badge variant="secondary">+3</Badge>
                    </div>
                    <p className="text-sm text-muted-foreground">Active Courses</p>
                    <p className="text-3xl font-bold">5</p>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="w-12 h-12 rounded-full bg-accent/20 flex items-center justify-center">
                        <Video className="w-6 h-6 text-accent" />
                      </div>
                      <Badge variant="secondary">+24%</Badge>
                    </div>
                    <p className="text-sm text-muted-foreground">Total Sessions</p>
                    <p className="text-3xl font-bold">234</p>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="w-12 h-12 rounded-full bg-purple-500/20 flex items-center justify-center">
                        <DollarSign className="w-6 h-6 text-purple-500" />
                      </div>
                      <Badge variant="secondary">+18%</Badge>
                    </div>
                    <p className="text-sm text-muted-foreground">Total Earnings</p>
                    <p className="text-3xl font-bold">$77,815</p>
                  </CardContent>
                </Card>
              </div>

              <div className="grid md:grid-cols-2 gap-6">
                {/* Upcoming Sessions */}
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Upcoming Sessions</CardTitle>
                    <CardDescription>Your scheduled live sessions</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {upcomingSessions.map((session) => (
                        <div key={session.id} className="p-3 rounded-lg bg-muted/30 hover:bg-muted/50 transition-colors">
                          <div className="flex items-center justify-between mb-2">
                            <div className="flex items-center gap-2">
                              <Video className="w-4 h-4 text-accent" />
                              <h4 className="font-semibold text-sm">{session.title}</h4>
                            </div>
                            <Badge>{session.registered} registered</Badge>
                          </div>
                          <p className="text-xs text-muted-foreground">{session.date}</p>
                          <p className="text-sm">{session.course}</p>
                        </div>
                      ))}
                    </div>
                    <Button className="w-full mt-4" variant="outline">
                      View Full Schedule
                      <ChevronRight className="w-4 h-4 ml-2" />
                    </Button>
                  </CardContent>
                </Card>

                {/* Student Inquiries */}
                <Card className="glass">
                  <CardHeader>
                    <div className="flex items-center justify-between">
                      <div>
                        <CardTitle>Student Inquiries</CardTitle>
                        <CardDescription>Pending questions from students</CardDescription>
                      </div>
                      <Badge>7 unanswered</Badge>
                    </div>
                  </CardHeader>
                  <CardContent>
                    <ScrollArea className="h-64">
                      <div className="space-y-3">
                        {studentInquiries.map((inquiry) => (
                          <div key={inquiry.id} className="p-3 rounded-lg bg-muted/30 hover:bg-muted/50 transition-colors">
                            <div className="flex items-start gap-2 mb-2">
                              <div className="w-8 h-8 rounded-full bg-primary/20 flex items-center justify-center flex-shrink-0">
                                <span className="text-xs font-bold">{inquiry.student.split(' ').map(n => n[0]).join('')}</span>
                              </div>
                              <div className="flex-1">
                                <p className="text-sm font-medium">{inquiry.student}</p>
                                <p className="text-xs text-muted-foreground">{inquiry.course}</p>
                              </div>
                              <Badge variant={inquiry.status === 'Unanswered' ? 'destructive' : 'secondary'}>
                                {inquiry.status}
                              </Badge>
                            </div>
                            <p className="text-sm">{inquiry.subject}</p>
                            <p className="text-xs text-muted-foreground">{inquiry.time}</p>
                          </div>
                        ))}
                      </div>
                    </ScrollArea>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            {/* My Courses Tab */}
            <TabsContent value="courses">
              <Card className="glass">
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <div>
                      <CardTitle>My Courses</CardTitle>
                      <CardDescription>Manage your published courses</CardDescription>
                    </div>
                    <Button>
                      <Plus className="w-4 h-4 mr-2" />
                      Create Course
                    </Button>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="grid md:grid-cols-2 gap-6">
                    {myCourses.map((course) => (
                      <Card key={course.id} className="hover:border-primary/50 transition-all duration-300">
                        <CardHeader>
                          <div className="flex items-center justify-between mb-2">
                            <Badge variant={course.status === 'Published' ? 'default' : 'secondary'}>
                              {course.status}
                            </Badge>
                            <div className="flex gap-2">
                              <Button variant="ghost" size="sm">
                                <Edit className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm">
                                <FileText className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm">
                                <Trash2 className="w-4 h-4" />
                              </Button>
                            </div>
                          </div>
                          <CardTitle className="text-xl">{course.title}</CardTitle>
                        </CardHeader>
                        <CardContent>
                          <div className="space-y-4">
                            <div className="grid grid-cols-2 gap-4 text-sm">
                              <div>
                                <p className="text-muted-foreground">Students</p>
                                <p className="font-semibold text-lg">{course.students.toLocaleString()}</p>
                              </div>
                              <div>
                                <p className="text-muted-foreground">Revenue</p>
                                <p className="font-semibold text-lg text-green-500">{course.revenue}</p>
                              </div>
                            </div>
                            <div>
                              <p className="text-sm text-muted-foreground mb-2">Completion Rate</p>
                              <div className="flex justify-between text-sm mb-1">
                                <span>Course Progress</span>
                                <span className="font-semibold">{course.progress}%</span>
                              </div>
                              <Progress value={course.progress} />
                            </div>
                            <div className="flex gap-2">
                              <Button className="flex-1" size="sm">
                                <Play className="w-4 h-4 mr-2" />
                                Manage
                              </Button>
                              <Button className="flex-1" size="sm" variant="outline">
                                <Video className="w-4 h-4 mr-2" />
                                Schedule
                              </Button>
                            </div>
                          </div>
                        </CardContent>
                      </Card>
                    ))}
                  </div>
                </CardContent>
              </Card>
            </TabsContent>

            {/* Students Tab */}
            <TabsContent value="students">
              <Card className="glass">
                <CardHeader>
                  <CardTitle>Student Management</CardTitle>
                  <CardDescription>View and manage your enrolled students</CardDescription>
                </CardHeader>
                <CardContent>
                  <ScrollArea className="h-96">
                    <div className="space-y-3">
                      {[1, 2, 3, 4, 5, 6, 7, 8].map((i) => (
                        <div key={i} className="flex items-center gap-4 p-4 rounded-lg bg-muted/30 hover:bg-muted/50 transition-colors">
                          <div className="w-12 h-12 rounded-full bg-primary/20 flex items-center justify-center flex-shrink-0">
                            <span className="text-lg font-bold">JD</span>
                          </div>
                          <div className="flex-1">
                            <div className="flex items-center justify-between mb-1">
                              <h4 className="font-semibold">John Doe {i}</h4>
                              <Badge variant="secondary">Active</Badge>
                            </div>
                            <p className="text-sm text-muted-foreground">john.doe{i}@example.com</p>
                            <div className="flex items-center gap-4 text-sm mt-2">
                              <span className="flex items-center gap-1">
                                <BookOpen className="w-4 h-4" />
                                3 Courses
                              </span>
                              <span className="flex items-center gap-1">
                                <TrendingUp className="w-4 h-4" />
                                75% Progress
                              </span>
                              <span className="flex items-center gap-1">
                                <Award className="w-4 h-4" />
                                2 Certificates
                              </span>
                            </div>
                          </div>
                          <div className="flex gap-2">
                            <Button size="sm" variant="outline">View Profile</Button>
                            <Button size="sm">Message</Button>
                          </div>
                        </div>
                      ))}
                    </div>
                  </ScrollArea>
                </CardContent>
              </Card>
            </TabsContent>

            {/* Schedule Tab */}
            <TabsContent value="schedule">
              <div className="grid md:grid-cols-2 gap-6">
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Schedule Session</CardTitle>
                    <CardDescription>Create a new live session</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <div className="space-y-2">
                        <label className="text-sm font-medium">Session Title</label>
                        <input
                          type="text"
                          placeholder="Enter session title"
                          className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm"
                        />
                      </div>
                      <div className="space-y-2">
                        <label className="text-sm font-medium">Select Course</label>
                        <select className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm">
                          <option>Unitree G1 Fundamentals</option>
                          <option>Python for Robotics</option>
                          <option>Control Systems</option>
                        </select>
                      </div>
                      <div className="space-y-2">
                        <label className="text-sm font-medium">Date & Time</label>
                        <input
                          type="datetime-local"
                          className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm"
                        />
                      </div>
                      <Button className="w-full">
                        <Video className="w-4 h-4 mr-2" />
                        Schedule Session
                      </Button>
                    </div>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Upcoming Sessions</CardTitle>
                    <CardDescription>Your scheduled live classes</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {upcomingSessions.map((session) => (
                        <div key={session.id} className="p-4 rounded-lg bg-muted/30">
                          <div className="flex items-center justify-between mb-2">
                            <div className="flex items-center gap-2">
                              <Video className="w-4 h-4 text-accent" />
                              <h4 className="font-semibold">{session.title}</h4>
                            </div>
                            <Badge variant="secondary">{session.registered} registered</Badge>
                          </div>
                          <p className="text-xs text-muted-foreground mb-1">{session.date}</p>
                          <p className="text-sm">{session.course}</p>
                          <div className="flex gap-2 mt-2">
                            <Button className="flex-1" size="sm">Start Session</Button>
                            <Button className="flex-1" size="sm" variant="outline">
                              <Edit className="w-4 h-4 mr-1" />
                              Edit
                            </Button>
                          </div>
                        </div>
                      ))}
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            {/* Earnings Tab */}
            <TabsContent value="earnings">
              <div className="grid md:grid-cols-2 gap-6">
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Earnings Overview</CardTitle>
                    <CardDescription>Your revenue breakdown</CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="p-4 rounded-lg bg-primary/10">
                      <p className="text-sm text-muted-foreground">Total Earnings</p>
                      <p className="text-4xl font-bold text-primary">$77,815</p>
                    </div>
                    <div className="grid grid-cols-2 gap-4">
                      <div className="p-4 rounded-lg bg-green-500/10">
                        <p className="text-sm text-muted-foreground">This Month</p>
                        <p className="text-2xl font-bold text-green-500">$8,450</p>
                      </div>
                      <div className="p-4 rounded-lg bg-accent/10">
                        <p className="text-sm text-muted-foreground">Last Month</p>
                        <p className="text-2xl font-bold">$7,230</p>
                      </div>
                    </div>
                    <div className="p-4 rounded-lg bg-purple-500/10">
                      <p className="text-sm text-muted-foreground">Average per Course</p>
                      <p className="text-2xl font-bold text-purple-500">$25,938</p>
                    </div>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Recent Payouts</CardTitle>
                    <CardDescription>Payment history</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <ScrollArea className="h-80">
                      <div className="space-y-3">
                        {[1, 2, 3, 4, 5].map((i) => (
                          <div key={i} className="flex items-center justify-between p-4 rounded-lg bg-muted/30">
                            <div>
                              <p className="font-semibold">Payout #{1000 - i}</p>
                              <p className="text-xs text-muted-foreground">
                                {(new Date(Date.now() - i * 7 * 24 * 60 * 60 * 1000)).toLocaleDateString()}
                              </p>
                            </div>
                            <Badge variant="default" className="text-green-500">
                              ${['8,450', '7,230', '6,180', '7,890', '8,120'][i]}
                            </Badge>
                          </div>
                        ))}
                      </div>
                    </ScrollArea>
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
