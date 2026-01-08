'use client'

import { motion } from 'framer-motion'
import { AnimatedLogo } from '@/components/AnimatedLogo'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Cpu, LogIn, UserPlus, Mail, Lock, MonitorPlay, GraduationCap, Shield, Users } from 'lucide-react'
import Link from 'next/link'
import { useState, useEffect } from 'react'

export default function AuthPage() {
  const [isLoading, setIsLoading] = useState(false)
  const [defaultTab, setDefaultTab] = useState('login')
  
  // Check for tab parameter in URL
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const params = new URLSearchParams(window.location.search)
      const tab = params.get('tab')
      if (tab === 'register') {
        setDefaultTab('register')
      }
    }
  }, [])

  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault()
    setIsLoading(true)
    // Simulate login
    setTimeout(() => {
      window.location.href = '/student'
    }, 1000)
  }

  return (
    <div className="min-h-screen flex items-center justify-center bg-gradient-to-br from-background via-primary/5 to-background px-4">
      <div className="w-full max-w-6xl grid lg:grid-cols-2 gap-12">
        {/* Left Side - Branding */}
        <motion.div
          initial={{ opacity: 0, x: -20 }}
          animate={{ opacity: 1, x: 0 }}
          className="flex flex-col justify-center"
        >
          <Link href="/" className="flex items-center gap-3 mb-8">
            <AnimatedLogo size="lg" />
          </Link>

          <h1 className="text-4xl font-bold mb-4">
            Welcome to the Future of
            <br />
            <span className="gradient-text">Robotics Education</span>
          </h1>

          <p className="text-lg text-muted-foreground mb-8">
            Join our comprehensive platform with personalized learning paths, real robot simulations, and expert instructors.
          </p>

          <div className="space-y-4">
            <div className="flex items-start gap-4">
              <div className="w-10 h-10 rounded-lg bg-primary/20 flex items-center justify-center flex-shrink-0">
                <GraduationCap className="w-5 h-5 text-primary" />
              </div>
              <div>
                <h3 className="font-semibold mb-1">For Students</h3>
                <p className="text-sm text-muted-foreground">
                  Access courses, track progress, join live sessions, and interact with instructors
                </p>
              </div>
            </div>

            <div className="flex items-start gap-4">
              <div className="w-10 h-10 rounded-lg bg-accent/20 flex items-center justify-center flex-shrink-0">
                <Users className="w-5 h-5 text-accent" />
              </div>
              <div>
                <h3 className="font-semibold mb-1">For Instructors</h3>
                <p className="text-sm text-muted-foreground">
                  Create and manage courses, host live sessions, track student performance
                </p>
              </div>
            </div>

            <div className="flex items-start gap-4">
              <div className="w-10 h-10 rounded-lg bg-purple-500/20 flex items-center justify-center flex-shrink-0">
                <Shield className="w-5 h-5 text-purple-500" />
              </div>
              <div>
                <h3 className="font-semibold mb-1">For Administrators</h3>
                <p className="text-sm text-muted-foreground">
                  Full platform management, analytics, user management, and system configuration
                </p>
              </div>
            </div>
          </div>
        </motion.div>

        {/* Right Side - Auth Forms */}
        <motion.div
          initial={{ opacity: 0, x: 20 }}
          animate={{ opacity: 1, x: 0 }}
        >
          <Card className="glass border-border/50">
            <CardHeader className="text-center">
              <CardTitle className="text-2xl">Sign In</CardTitle>
              <CardDescription>
                Choose your portal to access
              </CardDescription>
            </CardHeader>
            <CardContent>
              <Tabs defaultValue={defaultTab} value={defaultTab} onValueChange={setDefaultTab} className="w-full">
                <TabsList className="grid w-full grid-cols-2 mb-6">
                  <TabsTrigger value="login">Login</TabsTrigger>
                  <TabsTrigger value="register">Register</TabsTrigger>
                </TabsList>

                <TabsContent value="login">
                  <form onSubmit={handleLogin} className="space-y-4">
                    <div className="space-y-2">
                      <Label htmlFor="email">Email</Label>
                      <div className="relative">
                        <Mail className="absolute left-3 top-1/2 transform -translate-y-1/2 w-5 h-5 text-muted-foreground" />
                        <Input
                          id="email"
                          type="email"
                          placeholder="name@example.com"
                          className="pl-10"
                          required
                        />
                      </div>
                    </div>

                    <div className="space-y-2">
                      <Label htmlFor="password">Password</Label>
                      <div className="relative">
                        <Lock className="absolute left-3 top-1/2 transform -translate-y-1/2 w-5 h-5 text-muted-foreground" />
                        <Input
                          id="password"
                          type="password"
                          placeholder="••••••••••"
                          className="pl-10"
                          required
                        />
                      </div>
                    </div>

                    <div className="space-y-2">
                      <Label htmlFor="role">Portal</Label>
                      <select
                        id="role"
                        className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm ring-offset-background"
                      >
                        <option value="student">Student Portal</option>
                        <option value="provider">Provider/Instructor Portal</option>
                        <option value="admin">Admin Portal</option>
                      </select>
                    </div>

                    <Button type="submit" className="w-full" size="lg" disabled={isLoading}>
                      {isLoading ? 'Signing in...' : 'Sign In'}
                      <LogIn className="w-4 h-4 ml-2" />
                    </Button>
                  </form>

                  <div className="text-center space-y-4 pt-4">
                    <Link href="/auth/forgot-password" className="text-sm text-primary hover:underline">
                      Forgot password?
                    </Link>
                  </div>
                </TabsContent>

                <TabsContent value="register">
                  <form className="space-y-4">
                    <div className="grid grid-cols-2 gap-4">
                      <div className="space-y-2">
                        <Label htmlFor="firstName">First Name</Label>
                        <Input id="firstName" placeholder="John" required />
                      </div>
                      <div className="space-y-2">
                        <Label htmlFor="lastName">Last Name</Label>
                        <Input id="lastName" placeholder="Doe" required />
                      </div>
                    </div>

                    <div className="space-y-2">
                      <Label htmlFor="regEmail">Email</Label>
                      <div className="relative">
                        <Mail className="absolute left-3 top-1/2 transform -translate-y-1/2 w-5 h-5 text-muted-foreground" />
                        <Input
                          id="regEmail"
                          type="email"
                          placeholder="name@example.com"
                          className="pl-10"
                          required
                        />
                      </div>
                    </div>

                    <div className="space-y-2">
                      <Label htmlFor="regPassword">Password</Label>
                      <div className="relative">
                        <Lock className="absolute left-3 top-1/2 transform -translate-y-1/2 w-5 h-5 text-muted-foreground" />
                        <Input
                          id="regPassword"
                          type="password"
                          placeholder="••••••••••"
                          className="pl-10"
                          required
                        />
                      </div>
                    </div>

                    <div className="space-y-2">
                      <Label htmlFor="regRole">I am a</Label>
                      <select
                        id="regRole"
                        className="flex h-10 w-full rounded-md border border-input bg-background px-3 py-2 text-sm ring-offset-background"
                      >
                        <option value="student">Student</option>
                        <option value="provider">Instructor/Provider</option>
                      </select>
                    </div>

                    <Button type="submit" className="w-full" size="lg">
                      Create Account
                      <UserPlus className="w-4 h-4 ml-2" />
                    </Button>
                  </form>
                </TabsContent>
              </Tabs>
            </CardContent>
          </Card>

          {/* Quick Access */}
          <div className="mt-8 space-y-4">
            <h3 className="text-sm font-semibold text-muted-foreground uppercase tracking-wider">
              Quick Access
            </h3>
            <div className="grid grid-cols-2 gap-4">
              <Link href="/student">
                <Card className="glass hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                  <CardContent className="p-6 text-center">
                    <GraduationCap className="w-8 h-8 text-primary mx-auto mb-3" />
                    <h4 className="font-semibold group-hover:text-primary transition-colors">Student Portal</h4>
                    <p className="text-xs text-muted-foreground mt-1">Courses, Progress, Schedule</p>
                  </CardContent>
                </Card>
              </Link>

              <Link href="/provider">
                <Card className="glass hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                  <CardContent className="p-6 text-center">
                    <Users className="w-8 h-8 text-accent mx-auto mb-3" />
                    <h4 className="font-semibold group-hover:text-accent transition-colors">Instructor Portal</h4>
                    <p className="text-xs text-muted-foreground mt-1">Manage Courses, Students</p>
                  </CardContent>
                </Card>
              </Link>

              <Link href="/admin">
                <Card className="glass hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                  <CardContent className="p-6 text-center">
                    <Shield className="w-8 h-8 text-purple-500 mx-auto mb-3" />
                    <h4 className="font-semibold group-hover:text-purple-500 transition-colors">Admin Portal</h4>
                    <p className="text-xs text-muted-foreground mt-1">Platform Management</p>
                  </CardContent>
                </Card>
              </Link>

              <Link href="/video">
                <Card className="glass hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                  <CardContent className="p-6 text-center">
                    <MonitorPlay className="w-8 h-8 text-green-500 mx-auto mb-3" />
                    <h4 className="font-semibold group-hover:text-green-500 transition-colors">Video Portal</h4>
                    <p className="text-xs text-muted-foreground mt-1">Live Sessions & Meetings</p>
                  </CardContent>
                </Card>
              </Link>
            </div>
          </div>
        </motion.div>
      </div>
    </div>
  )
}
