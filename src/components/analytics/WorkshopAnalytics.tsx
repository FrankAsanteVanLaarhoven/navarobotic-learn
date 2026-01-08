/**
 * Workshop Analytics Component
 * Displays real-time student progress, performance metrics, and workshop insights
 */

'use client'

import { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Progress } from '@/components/ui/progress'
import { Badge } from '@/components/ui/badge'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import {
  Users, TrendingUp, Award, Clock, BookOpen,
  Trophy, Zap, Target, BarChart3, PieChart
} from 'lucide-react'

interface StudentProgress {
  id: string
  name: string
  email: string
  avatar: string
  courses: CourseProgress[]
  totalStudyTime: number
  lastActive: Date
  completionRate: number
  averageScore: number
  streakDays: number
  achievements: Achievement[]
}

interface CourseProgress {
  id: string
  title: string
  progress: number
  totalLessons: number
  completedLessons: number
  averageScore: number
  lastActive: Date
}

interface Achievement {
  id: string
  title: string
  description: string
  icon: string
  earnedAt: Date
}

interface WorkshopStats {
  totalStudents: number
  activeStudents: number
  completionRate: number
  averageCompletionTime: number
  totalSessions: number
  totalStudyHours: number
}

export function WorkshopAnalytics() {
  const [activeTab, setActiveTab] = useState('overview')
  const [workshopStats, setWorkshopStats] = useState<WorkshopStats>({
    totalStudents: 50,
    activeStudents: 32,
    completionRate: 78,
    averageCompletionTime: 240,
    totalSessions: 150,
    totalStudyHours: 1200
  })

  const [students] = useState<StudentProgress[]>([
    {
      id: '1',
      name: 'Alice Martinez',
      email: 'alice.martinez@email.com',
      avatar: 'AM',
      courses: [
        { id: 'ros2-basics', title: 'ROS2 Basics', progress: 85, totalLessons: 20, completedLessons: 17, averageScore: 92, lastActive: new Date() },
        { id: 'autonomous-systems', title: 'Autonomous Systems', progress: 60, totalLessons: 25, completedLessons: 15, averageScore: 85, lastActive: new Date(Date.now() - 86400000) }
      ],
      totalStudyTime: 12540,
      lastActive: new Date(),
      completionRate: 82,
      averageScore: 89,
      streakDays: 7,
      achievements: [
        { id: '1', title: 'Quick Learner', description: 'Completed 5 lessons in one day', icon: '🚀', earnedAt: new Date() }
      ]
    },
    {
      id: '2',
      name: 'Bob Kim',
      email: 'bob.kim@email.com',
      avatar: 'BK',
      courses: [
        { id: 'ai-robotics', title: 'AI-Powered Robotics', progress: 70, totalLessons: 30, completedLessons: 21, averageScore: 88, lastActive: new Date(Date.now() - 3600000) }
      ],
      totalStudyTime: 8940,
      lastActive: new Date(Date.now() - 1800000),
      completionRate: 75,
      averageScore: 85,
      streakDays: 3,
      achievements: []
    }
  ])

  const getOverallStats = () => {
    const allCourses = students.flatMap(s => s.courses)
    const totalLessons = allCourses.reduce((sum, c) => sum + c.totalLessons, 0)
    const completedLessons = allCourses.reduce((sum, c) => sum + c.completedLessons, 0)
    const avgCompletion = (completedLessons / totalLessons) * 100

    return {
      totalLessons,
      completedLessons,
      avgCompletion,
      totalCourses: allCourses.length
    }
  }

  return (
    <div className="min-h-screen bg-background">
      <div className="p-8">
        <div className="mb-8">
          <h1 className="text-3xl font-bold mb-2">Workshop Analytics</h1>
          <p className="text-muted-foreground">Real-time student progress, performance metrics, and workshop insights</p>
        </div>

        <Tabs defaultValue="overview" className="w-full" onValueChange={setActiveTab}>
          <TabsList className="grid grid-cols-4 w-full">
            <TabsTrigger value="overview">Overview</TabsTrigger>
            <TabsTrigger value="students">Students</TabsTrigger>
            <TabsTrigger value="courses">Courses</TabsTrigger>
            <TabsTrigger value="achievements">Achievements</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="mt-8">
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              <Card className="glass border-border/50">
                <CardHeader>
                  <CardTitle>Key Metrics</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="space-y-6">
                    <div className="flex items-center gap-4">
                      <div className="p-3 bg-primary/20 rounded-xl">
                        <Users className="w-8 h-8 text-primary" />
                      </div>
                      <div>
                        <div className="text-4xl font-bold">{workshopStats.totalStudents}</div>
                        <div className="text-sm text-muted-foreground">Total Students</div>
                      </div>
                      <Badge className="bg-primary/20 text-primary border-primary/50">
                        <TrendingUp className="w-4 h-4 mr-1" />
                        +5 this week
                      </Badge>
                    </div>

                    <div className="flex items-center gap-4">
                      <div className="p-3 bg-secondary/20 rounded-xl">
                        <Clock className="w-8 h-8 text-secondary" />
                      </div>
                      <div>
                        <div className="text-4xl font-bold">{workshopStats.averageCompletionTime}m</div>
                        <div className="text-sm text-muted-foreground">Avg. Completion Time</div>
                      </div>
                      <Badge className="bg-secondary/20 text-secondary border-secondary/50">
                        <Zap className="w-4 h-4 mr-1" />
                        10% faster
                      </Badge>
                    </div>

                    <div className="flex items-center gap-4">
                      <div className="p-3 bg-accent/20 rounded-xl">
                        <Award className="w-8 h-8 text-accent" />
                      </div>
                      <div>
                        <div className="text-4xl font-bold">{workshopStats.completionRate}%</div>
                        <div className="text-sm text-muted-foreground">Overall Completion Rate</div>
                      </div>
                      <Badge className="bg-accent/20 text-accent border-accent/50">
                        <TrendingUp className="w-4 h-4 mr-1" />
                        +3% this month
                      </Badge>
                    </div>
                  </div>
                </CardContent>
              </Card>

              <Card className="glass border-border/50">
                <CardHeader>
                  <CardTitle>Course Performance</CardTitle>
                  <CardDescription>Overview of all courses in workshop</CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="space-y-4">
                    {getOverallStats().totalCourses > 0 && (
                      <>
                        <div className="flex items-center justify-between mb-4">
                          <div className="text-lg font-semibold">Completion Overview</div>
                          <div className="flex items-center gap-2 text-sm text-muted-foreground">
                            <span>{getOverallStats().completedLessons}</span>
                            <span>/</span>
                            <span>{getOverallStats().totalLessons}</span>
                            <span>lessons</span>
                          </div>
                        </div>
                        <Progress value={getOverallStats().avgCompletion} className="w-full h-3" />
                      </>
                    )}

                    <div className="space-y-3">
                      <div className="flex items-center justify-between">
                        <div className="text-sm text-muted-foreground">Total Study Hours</div>
                        <div className="text-xl font-bold">{(workshopStats.totalStudyHours / 60).toFixed(1)}h</div>
                      </div>
                      <div className="flex items-center justify-between">
                        <div className="text-sm text-muted-foreground">Avg. Student Score</div>
                        <div className="text-xl font-bold">87.5%</div>
                      </div>
                    </div>
                  </div>
                </CardContent>
              </Card>
            </div>
          </TabsContent>

          <TabsContent value="students" className="mt-8">
            <Card className="glass border-border/50">
              <CardHeader>
                <CardTitle>Student Progress</CardTitle>
                <div className="flex items-center gap-2">
                  <Badge className="bg-primary/20 text-primary border-primary/50">
                    {workshopStats.activeStudents} Active
                  </Badge>
                  <Badge className="bg-secondary/20 text-secondary border-secondary/50">
                    {workshopStats.totalStudents} Total
                  </Badge>
                </div>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  {students.map((student) => (
                    <motion.div
                      key={student.id}
                      initial={{ opacity: 0, y: 20 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ duration: 0.3 }}
                    >
                      <div className="p-4 rounded-lg bg-muted/50 hover:bg-muted border border-border/50 transition-colors">
                        <div className="flex items-start gap-4 mb-3">
                          <div className="w-12 h-12 rounded-full bg-primary/20 flex items-center justify-center">
                            <span className="text-xl font-bold text-primary">{student.avatar}</span>
                          </div>
                          <div className="flex-1">
                            <div className="flex items-center justify-between mb-1">
                              <div className="font-semibold">{student.name}</div>
                              <Badge className={student.streakDays >= 7 ? 'bg-accent/20 text-accent border-accent/50' : 'bg-muted text-muted-foreground'}>
                                {student.streakDays} day streak
                              </Badge>
                            </div>
                            <div className="text-xs text-muted-foreground">
                              Last active: {new Date(student.lastActive).toLocaleString()}
                            </div>
                          </div>
                        </div>

                        <div className="space-y-3 mt-4">
                          {student.courses.slice(0, 3).map((course) => (
                            <div key={course.id} className="space-y-2">
                              <div className="flex items-center justify-between">
                                <div className="flex items-center gap-2">
                                  <BookOpen className="w-4 h-4 text-muted-foreground" />
                                  <span className="text-sm font-medium">{course.title}</span>
                                </div>
                                <div className="flex items-center gap-2">
                                  <span className="text-xs text-muted-foreground">{course.completedLessons}/{course.totalLessons}</span>
                                  <span className="text-sm font-bold">{course.progress}%</span>
                                </div>
                              </div>
                              <Progress value={course.progress} className="w-full h-2" />
                            </div>
                          ))}
                        </div>

                        <div className="grid grid-cols-3 gap-4 mt-4 pt-4 border-t border-border/50">
                          <div className="text-center">
                            <div className="text-2xl font-bold text-primary mb-1">{student.totalStudyTime / 60}h</div>
                            <div className="text-xs text-muted-foreground">Total Study</div>
                          </div>
                          <div className="text-center">
                            <div className="text-2xl font-bold text-secondary mb-1">{student.completionRate}%</div>
                            <div className="text-xs text-muted-foreground">Completion</div>
                          </div>
                          <div className="text-center">
                            <div className="text-2xl font-bold text-accent mb-1">{student.averageScore}%</div>
                            <div className="text-xs text-muted-foreground">Avg. Score</div>
                          </div>
                        </div>
                      </div>
                    </motion.div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="courses" className="mt-8">
            <Card className="glass border-border/50">
              <CardHeader>
                <CardTitle>Course Analytics</CardTitle>
                <CardDescription>Performance metrics for all courses</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  {(() => {
                    const uniqueCourses = new Map()
                    students.forEach(student => {
                      student.courses.forEach(course => {
                        uniqueCourses.set(course.id, course)
                      })
                    })
                    return Array.from(uniqueCourses.values())
                  })().map(course => (
                    <motion.div
                      key={course.id}
                      initial={{ opacity: 0, y: 20 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ duration: 0.3, delay: 0.1 }}
                    >
                      <div className="p-4 rounded-lg bg-muted/50 border border-border/50 hover:border-primary/50 transition-colors">
                        <div className="flex items-start gap-4">
                          <div className="w-16 h-16 rounded-xl bg-primary/20 flex items-center justify-center">
                            <BookOpen className="w-8 h-8 text-primary" />
                          </div>
                          <div className="flex-1">
                            <div className="font-semibold mb-1">{course.title}</div>
                            <div className="text-sm text-muted-foreground">{course.totalLessons} lessons</div>
                          </div>
                        </div>

                        <div className="grid grid-cols-3 gap-4 mt-4">
                          <div className="p-3 bg-muted rounded-lg">
                            <div className="text-sm text-muted-foreground mb-1">Enrolled</div>
                            <div className="text-2xl font-bold">{course.title === 'ros2-basics' ? 12 : 8}</div>
                          </div>
                          <div className="p-3 bg-muted rounded-lg">
                            <div className="text-sm text-muted-foreground mb-1">Avg. Progress</div>
                            <div className="text-2xl font-bold text-primary">{course.progress}%</div>
                          </div>
                          <div className="p-3 bg-muted rounded-lg">
                            <div className="text-sm text-muted-foreground mb-1">Avg. Score</div>
                            <div className="text-2xl font-bold text-secondary">{course.averageScore}</div>
                          </div>
                        </div>
                      </div>
                    </motion.div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="achievements" className="mt-8">
            <Card className="glass border-border/50">
              <CardHeader>
                <CardTitle>Achievements</CardTitle>
                <CardDescription>Student achievements and rewards</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  {students.flatMap(s => s.achievements).slice(0, 8).map((achievement) => (
                    <motion.div
                      key={achievement.id}
                      initial={{ opacity: 0, scale: 0.9 }}
                      animate={{ opacity: 1, scale: 1 }}
                      transition={{ duration: 0.3, delay: 0.1 }}
                      className="p-4 rounded-lg bg-muted/50 hover:bg-muted border border-border/50 hover:border-accent/50 transition-all"
                    >
                      <div className="flex items-center gap-4">
                        <div className="w-14 h-14 rounded-xl bg-accent/20 flex items-center justify-center">
                          <span className="text-2xl">{achievement.icon}</span>
                        </div>
                        <div>
                          <div className="font-semibold mb-1">{achievement.title}</div>
                          <div className="text-sm text-muted-foreground">{achievement.description}</div>
                          <div className="text-xs text-muted-foreground mt-2">
                            Earned: {new Date(achievement.earnedAt).toLocaleDateString()}
                          </div>
                        </div>
                      </div>
                    </motion.div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </div>
    </div>
  )
}
