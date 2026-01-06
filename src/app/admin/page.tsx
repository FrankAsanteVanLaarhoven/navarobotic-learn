'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Progress } from '@/components/ui/progress'
import { Separator } from '@/components/ui/separator'
import { Input } from '@/components/ui/input'
import {
  Shield,
  Users,
  BookOpen,
  TrendingUp,
  DollarSign,
  AlertTriangle,
  CheckCircle2,
  XCircle,
  Settings,
  LogOut,
  Plus,
  Search,
  Filter,
  BarChart3,
  Database,
  Globe,
  MonitorPlay,
  MessageSquare,
  ChevronRight,
  Edit,
  Trash2,
  MoreHorizontal
} from 'lucide-react'
import Link from 'next/link'
import { useState } from 'react'

export default function AdminPortal() {
  const [activeTab, setActiveTab] = useState('overview')

  const stats = [
    { title: 'Total Users', value: '12,458', change: '+12%', icon: <Users className="w-5 h-5" />, color: 'blue' },
    { title: 'Active Courses', value: '48', change: '+8%', icon: <BookOpen className="w-5 h-5" />, color: 'green' },
    { title: 'Revenue', value: '$45,230', change: '+18%', icon: <DollarSign className="w-5 h-5" />, color: 'purple' },
    { title: 'Video Hours', value: '1,234', change: '+24%', icon: <MonitorPlay className="w-5 h-5" />, color: 'orange' },
  ]

  const recentActivities = [
    { user: 'John Doe', action: 'enrolled in', course: 'Unitree G1 Fundamentals', time: '2 min ago', status: 'success' },
    { user: 'Sarah Smith', action: 'completed', course: 'Python for Robotics', time: '15 min ago', status: 'success' },
    { user: 'Mike Johnson', action: 'reported issue in', course: 'Control Systems', time: '1 hour ago', status: 'warning' },
    { user: 'Emily Davis', action: 'requested refund for', course: 'ROS2 & AI Integration', time: '2 hours ago', status: 'error' },
  ]

  const users = [
    { id: 1, name: 'John Doe', email: 'john@example.com', role: 'Student', courses: 12, status: 'Active', lastActive: '2 min ago' },
    { id: 2, name: 'Sarah Smith', email: 'sarah@example.com', role: 'Instructor', courses: 8, status: 'Active', lastActive: '1 hour ago' },
    { id: 3, name: 'Mike Johnson', email: 'mike@example.com', role: 'Student', courses: 5, status: 'Inactive', lastActive: '3 days ago' },
    { id: 4, name: 'Emily Davis', email: 'emily@example.com', role: 'Admin', courses: '-', status: 'Active', lastActive: '5 min ago' },
  ]

  const courses = [
    { id: 1, title: 'Unitree G1 Fundamentals', instructor: 'Dr. Smith', students: 2453, status: 'Published', revenue: '$12,265' },
    { id: 2, title: 'Python for Robotics', instructor: 'Prof. Johnson', students: 7234, status: 'Published', revenue: '$36,170' },
    { id: 3, title: 'Control Systems', instructor: 'Dr. Williams', students: 1892, status: 'Draft', revenue: '-' },
    { id: 4, title: 'ROS2 & AI Integration', instructor: 'Dr. Brown', students: 1245, status: 'Published', revenue: '$24,900' },
  ]

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-4">
              <Link href="/" className="flex items-center gap-2">
                <Shield className="h-8 w-8 text-primary" />
                <span className="text-lg font-bold gradient-text">ADMIN PORTAL</span>
              </Link>
            </div>

            <div className="flex items-center gap-4">
              <Button variant="ghost" size="sm">
                <MessageSquare className="w-4 h-4" />
                <Badge className="ml-2">3</Badge>
              </Button>
              <Button variant="ghost" size="sm">
                <AlertTriangle className="w-4 h-4" />
                <Badge variant="destructive" className="ml-2">5</Badge>
              </Button>
              <div className="flex items-center gap-2 pl-4 border-l border-border">
                <div className="w-8 h-8 rounded-full bg-purple-500/20 flex items-center justify-center">
                  <Shield className="w-4 h-4 text-purple-500" />
                </div>
                <div className="text-right">
                  <p className="text-sm font-medium">Admin User</p>
                  <p className="text-xs text-muted-foreground">Super Admin</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="pt-20 pb-8 px-4">
        <div className="container mx-auto">
          <Tabs value={activeTab} onValueChange={setActiveTab} className="w-full">
            <TabsList className="grid w-full max-w-3xl grid-cols-5 mb-6">
              <TabsTrigger value="overview">
                <BarChart3 className="w-4 h-4 mr-2" />
                Overview
              </TabsTrigger>
              <TabsTrigger value="users">
                <Users className="w-4 h-4 mr-2" />
                Users
              </TabsTrigger>
              <TabsTrigger value="courses">
                <BookOpen className="w-4 h-4 mr-2" />
                Courses
              </TabsTrigger>
              <TabsTrigger value="video">
                <MonitorPlay className="w-4 h-4 mr-2" />
                Video
              </TabsTrigger>
              <TabsTrigger value="settings">
                <Settings className="w-4 h-4 mr-2" />
                Settings
              </TabsTrigger>
            </TabsList>

            {/* Overview Tab */}
            <TabsContent value="overview">
              <div className="grid lg:grid-cols-4 gap-4 mb-8">
                {stats.map((stat, index) => (
                  <motion.div
                    key={index}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.1 }}
                  >
                    <Card className={`glass border-${stat.color}-500/30`}>
                      <CardContent className="p-6">
                        <div className="flex items-center justify-between mb-4">
                          <div className={`w-12 h-12 rounded-full bg-${stat.color}-500/20 flex items-center justify-center`}>
                            <div className={`text-${stat.color}-500`}>{stat.icon}</div>
                          </div>
                          <Badge variant={stat.change.startsWith('+') ? 'default' : 'destructive'}>
                            {stat.change}
                          </Badge>
                        </div>
                        <p className="text-sm text-muted-foreground">{stat.title}</p>
                        <p className="text-3xl font-bold">{stat.value}</p>
                      </CardContent>
                    </Card>
                  </motion.div>
                ))}
              </div>

              <div className="grid lg:grid-cols-2 gap-6">
                {/* Recent Activities */}
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Recent Activities</CardTitle>
                    <CardDescription>Latest user actions</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <ScrollArea className="h-80">
                      <div className="space-y-3">
                        {recentActivities.map((activity, index) => (
                          <div key={index} className="flex items-start gap-3 p-3 rounded-lg bg-muted/30">
                            <div className={`w-2 h-2 rounded-full flex-shrink-0 mt-2 ${
                              activity.status === 'success' ? 'bg-green-500' :
                              activity.status === 'warning' ? 'bg-yellow-500' :
                              'bg-red-500'
                            }`} />
                            <div className="flex-1">
                              <p className="text-sm">
                                <span className="font-medium">{activity.user}</span>
                                <span className="text-muted-foreground"> {activity.action} </span>
                                <span className="font-medium">{activity.course}</span>
                              </p>
                              <p className="text-xs text-muted-foreground">{activity.time}</p>
                            </div>
                          </div>
                        ))}
                      </div>
                    </ScrollArea>
                  </CardContent>
                </Card>

                {/* Quick Actions */}
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Quick Actions</CardTitle>
                    <CardDescription>Common administrative tasks</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-3">
                      <Link href="/admin/users">
                        <Button variant="outline" className="w-full justify-start">
                          <Users className="w-4 h-4 mr-2" />
                          Manage Users
                          <ChevronRight className="w-4 h-4 ml-auto" />
                        </Button>
                      </Link>
                      <Link href="/admin/courses">
                        <Button variant="outline" className="w-full justify-start">
                          <BookOpen className="w-4 h-4 mr-2" />
                          Manage Courses
                          <ChevronRight className="w-4 h-4 ml-auto" />
                        </Button>
                      </Link>
                      <Link href="/video">
                        <Button variant="outline" className="w-full justify-start">
                          <MonitorPlay className="w-4 h-4 mr-2" />
                          Video Sessions
                          <ChevronRight className="w-4 h-4 ml-auto" />
                        </Button>
                      </Link>
                      <Link href="/admin/settings">
                        <Button variant="outline" className="w-full justify-start">
                          <Settings className="w-4 h-4 mr-2" />
                          Platform Settings
                          <ChevronRight className="w-4 h-4 ml-auto" />
                        </Button>
                      </Link>
                      <Button variant="outline" className="w-full justify-start">
                        <Database className="w-4 h-4 mr-2" />
                        Generate Reports
                        <ChevronRight className="w-4 h-4 ml-auto" />
                      </Button>
                      <Button variant="outline" className="w-full justify-start">
                        <Globe className="w-4 h-4 mr-2" />
                          System Status
                        <ChevronRight className="w-4 h-4 ml-auto" />
                        </Button>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            {/* Users Tab */}
            <TabsContent value="users">
              <Card className="glass">
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <div>
                      <CardTitle>User Management</CardTitle>
                      <CardDescription>Manage all platform users</CardDescription>
                    </div>
                    <Button>
                      <Plus className="w-4 h-4 mr-2" />
                      Add User
                    </Button>
                  </div>
                </CardHeader>
                <CardContent>
                  <div className="flex gap-4 mb-4">
                    <div className="relative flex-1">
                      <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 w-5 h-5 text-muted-foreground" />
                      <Input placeholder="Search users..." className="pl-10" />
                    </div>
                    <Button variant="outline">
                      <Filter className="w-4 h-4 mr-2" />
                      Filter
                    </Button>
                  </div>

                  <ScrollArea className="h-96">
                    <table className="w-full">
                      <thead>
                        <tr className="border-b border-border/50">
                          <th className="text-left p-3 font-semibold">User</th>
                          <th className="text-left p-3 font-semibold">Email</th>
                          <th className="text-left p-3 font-semibold">Role</th>
                          <th className="text-left p-3 font-semibold">Courses</th>
                          <th className="text-left p-3 font-semibold">Status</th>
                          <th className="text-left p-3 font-semibold">Last Active</th>
                          <th className="text-right p-3 font-semibold">Actions</th>
                        </tr>
                      </thead>
                      <tbody>
                        {users.map((user, index) => (
                          <tr key={user.id} className="border-b border-border/50 hover:bg-muted/30">
                            <td className="p-3">
                              <div className="flex items-center gap-2">
                                <div className="w-8 h-8 rounded-full bg-primary/20 flex items-center justify-center">
                                  <span className="text-xs font-bold">{user.name.split(' ').map(n => n[0]).join('')}</span>
                                </div>
                                <span className="font-medium">{user.name}</span>
                              </div>
                            </td>
                            <td className="p-3 text-sm text-muted-foreground">{user.email}</td>
                            <td className="p-3">
                              <Badge variant={user.role === 'Admin' ? 'default' : user.role === 'Instructor' ? 'secondary' : 'outline'}>
                                {user.role}
                              </Badge>
                            </td>
                            <td className="p-3">{user.courses}</td>
                            <td className="p-3">
                              <Badge variant={user.status === 'Active' ? 'default' : 'secondary'}>
                                {user.status}
                              </Badge>
                            </td>
                            <td className="p-3 text-sm text-muted-foreground">{user.lastActive}</td>
                            <td className="p-3 text-right">
                              <Button variant="ghost" size="sm">
                                <Edit className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm">
                                <Trash2 className="w-4 h-4" />
                              </Button>
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </ScrollArea>
                </CardContent>
              </Card>
            </TabsContent>

            {/* Courses Tab */}
            <TabsContent value="courses">
              <Card className="glass">
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <div>
                      <CardTitle>Course Management</CardTitle>
                      <CardDescription>Manage all courses and content</CardDescription>
                    </div>
                    <Button>
                      <Plus className="w-4 h-4 mr-2" />
                      Create Course
                    </Button>
                  </div>
                </CardHeader>
                <CardContent>
                  <ScrollArea className="h-96">
                    <table className="w-full">
                      <thead>
                        <tr className="border-b border-border/50">
                          <th className="text-left p-3 font-semibold">Course</th>
                          <th className="text-left p-3 font-semibold">Instructor</th>
                          <th className="text-left p-3 font-semibold">Students</th>
                          <th className="text-left p-3 font-semibold">Status</th>
                          <th className="text-left p-3 font-semibold">Revenue</th>
                          <th className="text-right p-3 font-semibold">Actions</th>
                        </tr>
                      </thead>
                      <tbody>
                        {courses.map((course) => (
                          <tr key={course.id} className="border-b border-border/50 hover:bg-muted/30">
                            <td className="p-3 font-medium">{course.title}</td>
                            <td className="p-3 text-sm text-muted-foreground">{course.instructor}</td>
                            <td className="p-3">{course.students.toLocaleString()}</td>
                            <td className="p-3">
                              <Badge variant={course.status === 'Published' ? 'default' : 'secondary'}>
                                {course.status}
                              </Badge>
                            </td>
                            <td className="p-3 font-semibold">{course.revenue}</td>
                            <td className="p-3 text-right">
                              <Button variant="ghost" size="sm">
                                <Edit className="w-4 h-4" />
                              </Button>
                              <Button variant="ghost" size="sm">
                                <MoreHorizontal className="w-4 h-4" />
                              </Button>
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </ScrollArea>
                </CardContent>
              </Card>
            </TabsContent>

            {/* Video Tab */}
            <TabsContent value="video">
              <div className="grid md:grid-cols-2 gap-6">
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Active Sessions</CardTitle>
                    <CardDescription>Currently live video sessions</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      {[1, 2, 3].map((i) => (
                        <div key={i} className="p-4 rounded-lg bg-red-500/10 border border-red-500/30">
                          <div className="flex items-center justify-between mb-2">
                            <div className="flex items-center gap-2">
                              <span className="relative flex h-2 w-2">
                                <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-red-400 opacity-75"></span>
                                <span className="relative inline-flex rounded-full h-2 w-2 bg-red-500"></span>
                              </span>
                              <h4 className="font-semibold">Live Session {i}</h4>
                            </div>
                            <Badge>45 viewers</Badge>
                          </div>
                          <p className="text-sm text-muted-foreground mb-3">Control Systems - Module 3</p>
                          <div className="flex gap-2">
                            <Button size="sm">Join Session</Button>
                            <Button size="sm" variant="outline">Monitor</Button>
                          </div>
                        </div>
                      ))}
                    </div>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Session Analytics</CardTitle>
                    <CardDescription>Video platform statistics</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                        <span className="text-sm">Total Sessions Today</span>
                        <span className="font-bold text-2xl">24</span>
                      </div>
                      <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                        <span className="text-sm">Total Viewers</span>
                        <span className="font-bold text-2xl">1,234</span>
                      </div>
                      <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                        <span className="text-sm">Avg Duration</span>
                        <span className="font-bold text-2xl">45m</span>
                      </div>
                      <Button className="w-full">
                        View Detailed Analytics
                        <ChevronRight className="w-4 h-4 ml-2" />
                      </Button>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </TabsContent>

            {/* Settings Tab */}
            <TabsContent value="settings">
              <div className="grid lg:grid-cols-2 gap-6">
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Platform Settings</CardTitle>
                    <CardDescription>Configure system-wide settings</CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="space-y-2">
                      <label className="text-sm font-medium">Platform Name</label>
                      <Input defaultValue="Robotics Mastery" />
                    </div>
                    <div className="space-y-2">
                      <label className="text-sm font-medium">Support Email</label>
                      <Input defaultValue="support@roboticsmastery.com" />
                    </div>
                    <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                      <span className="text-sm font-medium">Maintenance Mode</span>
                      <Badge variant="secondary">Disabled</Badge>
                    </div>
                    <Button className="w-full">Save Changes</Button>
                  </CardContent>
                </Card>

                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Security Settings</CardTitle>
                    <CardDescription>Platform security configuration</CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                      <span className="text-sm font-medium">2FA Required</span>
                      <Badge variant="default">Enabled</Badge>
                    </div>
                    <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                      <span className="text-sm font-medium">Session Timeout</span>
                      <span className="font-medium">30 min</span>
                    </div>
                    <div className="flex items-center justify-between p-3 rounded-lg bg-muted/30">
                      <span className="text-sm font-medium">IP Whitelist</span>
                      <Button size="sm" variant="outline">Configure</Button>
                    </div>
                    <Button className="w-full">View Security Logs</Button>
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
