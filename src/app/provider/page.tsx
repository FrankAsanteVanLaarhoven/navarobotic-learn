'use client'

import { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Progress } from '@/components/ui/progress'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import {
  DollarSign, Users, Trophy, TrendingUp, Star,
  Eye, MessageSquare, Clock, Calendar,
  Play, Plus, Download, Bell, Search, Filter,
  BookOpen, Video, Code, Target, Award,
  ArrowUpRight, ArrowDown, MoreHorizontal, X,
  Settings, Activity, BarChart3, FileText
} from 'lucide-react'
import Link from 'next/link'
import { Cpu } from 'lucide-react'

const fadeInUp = {
  initial: { opacity: 0, y: 20 },
  animate: { opacity: 1, y: 0 },
  transition: { duration: 0.5 }
}

export default function ProviderDashboard() {
  const [mounted, setMounted] = useState(false)
  const [selectedPeriod, setSelectedPeriod] = useState('month')

  useEffect(() => {
    setMounted(true)
  }, [])

  const stats = {
    totalCourses: 8,
    totalStudents: 12540,
    totalEarnings: 89420,
    totalCertificates: 1560,
    activeStudents: 3240,
    completionRate: 78,
    averageRating: 4.8,
    revenueThisMonth: 12340,
    revenueGrowth: 15
  }

  const courses = [
    {
      id: 1,
      title: 'ROS2 Basics',
      students: 12540,
      rating: 4.9,
      completion: 36,
      revenue: 42560,
      enrollment: 'high',
      category: 'Foundation',
      status: 'active',
      lessons: 73,
      modules: 8
    },
    {
      id: 2,
      title: 'AI-Powered Robotics',
      students: 8920,
      rating: 4.8,
      completion: 42,
      revenue: 38520,
      enrollment: 'high',
      category: 'Advanced',
      status: 'active',
      lessons: 45,
      modules: 12
    },
    {
      id: 3,
      title: 'Autonomous Systems',
      students: 7650,
      rating: 4.7,
      completion: 65,
      revenue: 29680,
      enrollment: 'medium',
      category: 'Navigation',
      status: 'active',
      lessons: 52,
      modules: 10
    },
    {
      id: 4,
      title: 'Robotic Arm Programming',
      students: 15230,
      rating: 4.9,
      completion: 23,
      revenue: 32490,
      enrollment: 'high',
      category: 'Manipulation',
      status: 'active',
      lessons: 38,
      modules: 6
    }
  ]

  const recentActivity = [
    { id: 1, type: 'enrollment', icon: <Users className="w-4 h-4" />, user: 'John D.', course: 'ROS2 Basics', time: '2 min ago' },
    { id: 2, type: 'completion', icon: <Trophy className="w-4 h-4" />, user: 'Jane S.', course: 'AI-Powered Robotics', time: '15 min ago' },
    { id: 3, type: 'earnings', icon: <DollarSign className="w-4 h-4" />, user: 'System Revenue', course: 'Platform fee', time: '1 hr ago' },
    { id: 4, type: 'review', icon: <Star className="w-4 h-4" />, user: 'Mike R.', course: 'Autonomous Systems', time: '30 min ago', rating: 5 },
    { id: 5, type: 'announcement', icon: <Bell className="w-4 h-4" />, course: 'Platform Update', time: '2 hrs ago' }
  ]

  const revenueData = [
    { month: 'Jan', earnings: 8540 },
    { month: 'Feb', earnings: 10230 },
    { month: 'Mar', earnings: 9870 },
    { month: 'Apr', earnings: 12340 },
    { month: 'May', earnings: 11520 },
    { month: 'Jun', earnings: 9430 }
  ]

  const maxEarnings = Math.max(...revenueData.map(d => d.earnings))

  if (!mounted) return null

  return (
    <div className="min-h-screen bg-background">
      <div className="flex h-screen overflow-hidden">
        {/* Sidebar Navigation */}
        <aside className="w-64 border-r border-border/50 glass flex flex-col">
          <div className="p-4 border-b border-border/50">
            <div className="flex items-center gap-3">
              <div className="w-10 h-10 rounded-lg bg-primary/20 flex items-center justify-center">
                <img src="/logos/logo-icon.svg" alt="Rovyn Logo" className="w-6 h-6" />
              </div>
              <div>
                <h1 className="text-lg font-bold">Provider Dashboard</h1>
                <p className="text-xs text-muted-foreground">Manage your courses</p>
              </div>
            </div>
          </div>

          <nav className="flex-1 p-4 space-y-2 overflow-y-auto">
            <Link href="/provider" className="flex items-center gap-3 p-3 rounded-lg bg-primary/10 text-primary hover:bg-primary/20 transition-colors">
              <BarChart3 className="w-5 h-5" />
              <span className="font-semibold">Overview</span>
            </Link>
            <Link href="/provider/courses" className="flex items-center gap-3 p-3 rounded-lg text-muted-foreground hover:text-foreground hover:bg-muted/50 transition-colors">
              <BookOpen className="w-5 h-5" />
              <span className="font-medium">My Courses</span>
            </Link>
            <Link href="/provider/students" className="flex items-center gap-3 p-3 rounded-lg text-muted-foreground hover:text-foreground hover:bg-muted/50 transition-colors">
              <Users className="w-5 h-5" />
              <span className="font-medium">Students</span>
            </Link>
            <Link href="/provider/analytics" className="flex items-center gap-3 p-3 rounded-lg text-muted-foreground hover:text-foreground hover:bg-muted/50 transition-colors">
              <TrendingUp className="w-5 h-5" />
              <span className="font-medium">Analytics</span>
            </Link>
            <Link href="/provider/settings" className="flex items-center gap-3 p-3 rounded-lg text-muted-foreground hover:text-foreground hover:bg-muted/50 transition-colors">
              <Settings className="w-5 h-5" />
              <span className="font-medium">Settings</span>
            </Link>
            <Link href="/student" className="flex items-center gap-3 p-3 rounded-lg text-muted-foreground hover:text-foreground hover:bg-muted/50 transition-colors">
              <X className="w-5 h-5" />
              <span className="font-medium">Exit Provider View</span>
            </Link>
          </nav>

          {/* Profile Section */}
          <div className="p-4 border-t border-border/50">
            <div className="flex items-center gap-3 mb-4">
              <div className="w-12 h-12 rounded-full bg-primary/20 flex items-center justify-center">
                <Users className="w-6 h-6 text-primary" />
              </div>
              <div>
                <h3 className="text-lg font-semibold">Tech Institute</h3>
                <p className="text-xs text-muted-foreground">Premium Provider</p>
              </div>
            </div>
            <div className="space-y-2">
              <div className="flex items-center justify-between text-sm">
                <span className="text-muted-foreground">Rating</span>
                <div className="flex items-center gap-1">
                  <Star className="w-4 h-4 text-accent fill-accent" />
                  <span className="font-semibold">4.9</span>
                </div>
              </div>
              <div className="flex items-center justify-between text-sm">
                <span className="text-muted-foreground">Courses</span>
                <span className="font-semibold">8</span>
              </div>
              <div className="flex items-center justify-between text-sm">
                <span className="text-muted-foreground">Students</span>
                <span className="font-semibold">12,540</span>
              </div>
              <div className="flex items-center justify-between text-sm">
                <span className="text-muted-foreground">Earnings</span>
                <span className="font-semibold text-primary">$89,420</span>
              </div>
            </div>
          </div>
        </aside>

        {/* Main Content */}
        <main className="flex-1 overflow-y-auto">
          {/* Header */}
          <header className="border-b border-border/50 glass p-4">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-3">
                <h2 className="text-2xl font-bold">Provider Dashboard</h2>
                <Badge className="bg-primary/20 text-primary border-primary/50">
                  <Trophy className="w-3 h-3 mr-1" />
                  Premium Provider
                </Badge>
              </div>
              <div className="flex items-center gap-3">
                <div className="relative">
                  <Bell className="w-6 h-6 text-muted-foreground cursor-pointer hover:text-foreground transition-colors" />
                  <div className="absolute -top-1 -right-1 w-2 h-2 bg-destructive rounded-full" />
                </div>
                <div className="flex items-center gap-2 text-sm text-muted-foreground">
                  <Eye className="w-4 h-4" />
                  <span>Profile Views: <span className="font-semibold text-foreground">2,341</span></span>
                </div>
              </div>
            </div>
          </header>

          {/* Dashboard Content */}
          <div className="p-6 space-y-6">
            {/* Overview Stats Cards */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5 }}
            >
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
                <Card className="glass border-border/50">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-3">
                        <div className="p-2 bg-primary/20 rounded-lg">
                          <BookOpen className="w-6 h-6 text-primary" />
                        </div>
                        <span className="text-sm text-muted-foreground">Total Courses</span>
                      </div>
                    </div>
                    <div className="text-3xl font-bold">{stats.totalCourses}</div>
                    <div className="text-xs text-primary mt-1">+2 this month</div>
                  </CardContent>
                </Card>

                <Card className="glass border-border/50">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-3">
                        <div className="p-2 bg-secondary/20 rounded-lg">
                          <Users className="w-6 h-6 text-secondary" />
                        </div>
                        <span className="text-sm text-muted-foreground">Total Students</span>
                      </div>
                    </div>
                    <div className="text-3xl font-bold">{stats.totalStudents.toLocaleString()}</div>
                    <div className="text-xs text-secondary mt-1">+540 this month</div>
                  </CardContent>
                </Card>

                <Card className="glass border-border/50">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-3">
                        <div className="p-2 bg-primary/20 rounded-lg">
                          <DollarSign className="w-6 h-6 text-primary" />
                        </div>
                        <span className="text-sm text-muted-foreground">Total Earnings</span>
                      </div>
                    </div>
                    <div className="text-3xl font-bold">${stats.totalEarnings.toLocaleString()}</div>
                    <div className="text-xs text-primary mt-1">+${stats.revenueThisMonth.toLocaleString()} this month</div>
                  </CardContent>
                </Card>

                <Card className="glass border-border/50">
                  <CardContent className="p-6">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-3">
                        <div className="p-2 bg-accent/20 rounded-lg">
                          <Award className="w-6 h-6 text-accent" />
                        </div>
                        <span className="text-sm text-muted-foreground">Certificates</span>
                      </div>
                    </div>
                    <div className="text-3xl font-bold">{stats.totalCertificates}</div>
                    <div className="text-xs text-accent mt-1">156 issued</div>
                  </CardContent>
                </Card>
              </div>
            </motion.div>

            {/* Revenue Chart */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.1 }}
            >
              <Card className="glass border-border/50">
                <CardHeader className="flex items-center justify-between">
                  <div className="flex items-center gap-3">
                    <div className="p-2 bg-primary/20 rounded-lg">
                      <DollarSign className="w-6 h-6 text-primary" />
                    </div>
                    <div>
                      <CardTitle>Revenue Overview</CardTitle>
                      <CardDescription>Monthly earnings and growth</CardDescription>
                    </div>
                  </div>
                  <div className="flex items-center gap-2">
                    <Button
                      variant={selectedPeriod === 'month' ? 'default' : 'outline'}
                      size="sm"
                      onClick={() => setSelectedPeriod('month')}
                    >
                      Month
                    </Button>
                    <Button
                      variant={selectedPeriod === 'quarter' ? 'default' : 'outline'}
                      size="sm"
                      onClick={() => setSelectedPeriod('quarter')}
                    >
                      Quarter
                    </Button>
                    <Button
                      variant={selectedPeriod === 'year' ? 'default' : 'outline'}
                      size="sm"
                      onClick={() => setSelectedPeriod('year')}
                    >
                      Year
                    </Button>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="h-64 flex items-end justify-between gap-2 px-4 pb-4">
                    {revenueData.map((data, idx) => {
                      const barHeight = (data.earnings / maxEarnings) * 100
                      return (
                        <div key={idx} className="flex-1 flex flex-col items-center gap-2">
                          <div className="text-xs text-muted-foreground">{data.month}</div>
                          <div className="relative w-full h-48 bg-muted/50 rounded-t-lg overflow-hidden group">
                            <motion.div
                              className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-primary to-secondary rounded-t-lg"
                              initial={{ height: 0 }}
                              animate={{ height: `${barHeight}%` }}
                              transition={{ duration: 0.5, delay: idx * 0.1 }}
                            />
                            <div className="absolute top-2 left-2 right-2 z-10 opacity-0 group-hover:opacity-100 transition-opacity">
                              <div className="bg-card/90 backdrop-blur-sm rounded-lg p-2 border border-border/50">
                                <div className="text-sm font-semibold">${(data.earnings / 1000).toFixed(1)}K</div>
                              </div>
                            </div>
                          </div>
                          <div className="text-sm font-semibold">${(data.earnings / 1000).toFixed(1)}K</div>
                        </div>
                      )
                    })}
                  </div>
                  <div className="mt-6 flex items-center justify-center gap-4">
                    <div className="text-center">
                      <div className="text-3xl font-bold text-primary mb-1">{stats.revenueGrowth}%</div>
                      <div className="flex items-center gap-1 text-sm text-muted-foreground">
                        <TrendingUp className="w-4 h-4" />
                        <span>Growth Rate</span>
                      </div>
                    </div>
                  </div>
                </CardContent>
              </Card>
            </motion.div>

            {/* Course Performance Table */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.2 }}
            >
              <Card className="glass border-border/50">
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-3">
                      <div className="p-2 bg-secondary/20 rounded-lg">
                        <Trophy className="w-6 h-6 text-secondary" />
                      </div>
                      <div>
                        <CardTitle>Course Performance</CardTitle>
                        <CardDescription>Overview of your courses and their metrics</CardDescription>
                      </div>
                    </div>
                    <div className="flex items-center gap-2">
                      <Button variant="outline" size="sm">
                        <Filter className="w-4 h-4 mr-2" />
                        Filter
                      </Button>
                      <Button variant="outline" size="sm">
                        <Download className="w-4 h-4 mr-2" />
                        Export
                      </Button>
                    </div>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="overflow-x-auto">
                    <table className="w-full">
                      <thead>
                        <tr className="text-left border-b border-border/50">
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Course</th>
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Students</th>
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Rating</th>
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Completion</th>
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Revenue</th>
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Status</th>
                          <th className="p-4 text-xs font-semibold text-muted-foreground uppercase tracking-wider">Actions</th>
                        </tr>
                      </thead>
                      <tbody>
                        {courses.map((course) => (
                          <tr key={course.id} className="border-b border-border/50 hover:bg-muted/30 transition-colors">
                            <td className="p-4">
                              <div className="flex items-center gap-3">
                                <span className="font-semibold">{course.title}</span>
                                <Badge variant="outline" className="text-xs">
                                  {course.category}
                                </Badge>
                              </div>
                            </td>
                            <td className="p-4">
                              <div className="flex items-center gap-2">
                                <Users className="w-4 h-4 text-muted-foreground" />
                                <span className="font-semibold">{course.students.toLocaleString()}</span>
                              </div>
                            </td>
                            <td className="p-4">
                              <div className="flex items-center gap-2">
                                <Star className="w-4 h-4 text-accent fill-accent" />
                                <span className="font-semibold">{course.rating}</span>
                              </div>
                            </td>
                            <td className="p-4">
                              <div className="flex items-center gap-3">
                                <Progress value={course.completion} className="w-24 h-2" />
                                <span className="font-semibold ml-2">{course.completion}%</span>
                              </div>
                            </td>
                            <td className="p-4">
                              <div className="flex items-center gap-3">
                                <div className="p-2 bg-primary/20 rounded-lg">
                                  <DollarSign className="w-4 h-4 text-primary" />
                                </div>
                                <span className="font-semibold">${course.revenue.toLocaleString()}</span>
                              </div>
                            </td>
                            <td className="p-4">
                              <Badge className={course.status === 'active' ? 'bg-primary/20 text-primary border-primary/50' : 'bg-accent/20 text-accent border-accent/50'}>
                                {course.status}
                              </Badge>
                            </td>
                            <td className="p-4">
                              <div className="flex gap-2">
                                <Button variant="ghost" size="sm">
                                  <Eye className="w-4 h-4 mr-1" />
                                  View
                                </Button>
                                <Button variant="ghost" size="sm">
                                  <MoreHorizontal className="w-4 h-4" />
                                </Button>
                              </div>
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                </CardContent>
              </Card>
            </motion.div>

            {/* Student Engagement & Recent Activity */}
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: 0.3 }}
              >
                <Card className="glass border-border/50">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="p-2 bg-secondary/20 rounded-lg">
                        <Users className="w-6 h-6 text-secondary" />
                      </div>
                      <CardTitle>Student Engagement</CardTitle>
                    </div>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="flex items-center justify-between">
                      <div className="space-y-1">
                        <div className="text-sm text-muted-foreground">Active Students</div>
                        <div className="text-3xl font-bold">{stats.activeStudents.toLocaleString()}</div>
                      </div>
                      <Badge className="bg-secondary/20 text-secondary border-secondary/50">
                        <Activity className="w-4 h-4 mr-1" />
                        +12% from last month
                      </Badge>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="space-y-1">
                        <div className="text-sm text-muted-foreground">Completion Rate</div>
                        <div className="flex items-center gap-2">
                          <TrendingUp className="w-4 h-4 text-primary" />
                          <span className="text-3xl font-bold">{stats.completionRate}%</span>
                        </div>
                      </div>
                      <div className="text-xs text-primary">
                        Above platform average (67%)
                      </div>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="space-y-1">
                        <div className="text-sm text-muted-foreground">Average Rating</div>
                        <div className="flex items-center gap-2">
                          <Star className="w-5 h-5 text-accent fill-accent" />
                          <span className="text-3xl font-bold">{stats.averageRating}</span>
                        </div>
                      </div>
                      <div className="text-xs text-accent">
                        Top 10% of all providers
                      </div>
                    </div>

                    <div className="pt-4">
                      <Progress value={stats.completionRate} className="h-2" />
                    </div>
                  </CardContent>
                </Card>
              </motion.div>

              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: 0.4 }}
              >
                <Card className="glass border-border/50">
                  <CardHeader>
                    <div className="flex items-center gap-3">
                      <div className="p-2 bg-accent/20 rounded-lg">
                        <Clock className="w-6 h-6 text-accent" />
                      </div>
                      <CardTitle>Recent Activity</CardTitle>
                    </div>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      {recentActivity.map((activity) => {
                        const activityColors = {
                          enrollment: 'bg-primary/10 border-primary/30',
                          completion: 'bg-accent/10 border-accent/30',
                          earnings: 'bg-primary/10 border-primary/30',
                          review: 'bg-secondary/10 border-secondary/30',
                          announcement: 'bg-accent/10 border-accent/30'
                        }

                        return (
                          <div key={activity.id} className={`flex items-center gap-4 p-4 rounded-lg border ${activityColors[activity.type as keyof typeof activityColors]}`}>
                            <div className={`p-2 rounded-lg ${activityColors[activity.type as keyof typeof activityColors]}`}>
                              {activity.icon}
                            </div>
                            <div className="flex-1">
                              <div className="font-semibold mb-1">
                                {activity.type === 'enrollment' && 'New Enrollment'}
                                {activity.type === 'completion' && 'Course Completed'}
                                {activity.type === 'earnings' && 'Earnings Update'}
                                {activity.type === 'review' && `New Review: ${activity.rating}★`}
                                {activity.type === 'announcement' && 'Platform Update'}
                              </div>
                              <div className="text-sm text-muted-foreground">
                                {activity.user} {activity.type === 'enrollment' && 'enrolled in'}
                                {activity.type === 'completion' && 'completed'}
                                {activity.type === 'earnings' && 'earned from'}
                                {activity.type === 'review' && 'rated'}
                                {activity.type === 'announcement' && 'announced'} {activity.course}
                              </div>
                              <div className="text-xs text-muted-foreground mt-1">{activity.time}</div>
                            </div>
                          </div>
                        )
                      })}
                    </div>
                  </CardContent>
                </Card>
              </motion.div>
            </div>

            {/* Quick Actions */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.5 }}
            >
              <Card className="glass border-border/50 bg-primary/5">
                <CardHeader>
                  <div className="flex items-center gap-3">
                    <div className="p-2 bg-primary/20 rounded-lg">
                      <Play className="w-6 h-6 text-primary" />
                    </div>
                    <CardTitle>Quick Actions</CardTitle>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                    <Button className="w-full">
                      <Plus className="w-5 h-5 mr-2" />
                      Create New Course
                    </Button>
                    <Link href="/provider/courses" className="w-full">
                      <Button variant="outline" className="w-full">
                        <BookOpen className="w-5 h-5 mr-2" />
                        Manage Courses
                      </Button>
                    </Link>
                    <Link href="/provider/students" className="w-full">
                      <Button variant="outline" className="w-full">
                        <Users className="w-5 h-5 mr-2" />
                        View Students
                      </Button>
                    </Link>
                    <Button variant="outline" className="w-full">
                      <Bell className="w-5 h-5 mr-2" />
                      Send Announcement
                    </Button>
                  </div>
                </CardContent>
              </Card>
            </motion.div>
          </div>
        </main>
      </div>
    </div>
  )
}
