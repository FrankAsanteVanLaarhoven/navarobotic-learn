'use client'

import { motion } from 'framer-motion'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import {
  Play,
  Zap,
  Cpu,
  Brain,
  Rocket,
  Shield,
  Code,
  Target,
  Users,
  Award,
  ChevronRight,
  Microscope,
  Layout,
  Globe,
  Sparkles,
  Terminal,
  Box,
  MonitorPlay,
  BookOpen,
  Trophy,
  MessageCircle,
  Video,
  Headphones
} from 'lucide-react'
import Link from 'next/link'

const fadeInUp = {
  initial: { opacity: 0, y: 20 },
  animate: { opacity: 1, y: 0 },
  transition: { duration: 0.5 }
}

const staggerContainer = {
  animate: {
    transition: {
      staggerChildren: 0.1
    }
  }
}

export default function Home() {
  return (
    <div className="min-h-screen flex flex-col">
      {/* Navigation */}
      <nav className="fixed top-0 left-0 right-0 z-50 glass border-b border-border/50">
        <div className="container mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center gap-2">
              <Cpu className="h-8 w-8 text-primary" />
              <span className="text-xl font-bold gradient-text">ROBOTICS MASTERY</span>
            </div>
            <div className="hidden md:flex items-center gap-8">
              <Link href="/catalog" className="text-sm text-muted-foreground hover:text-foreground transition-colors">
                Courses
              </Link>
              <Link href="/catalog#paths" className="text-sm text-muted-foreground hover:text-foreground transition-colors">
                Learning Paths
              </Link>
              <Link href="/ai-video/generate" className="text-sm text-primary hover:text-primary transition-colors font-semibold">
                <Video className="w-4 h-4 inline mr-1" />
                AI Video Studio
              </Link>
              <Link href="/simulation" className="text-sm text-muted-foreground hover:text-foreground transition-colors font-semibold">
                <Box className="w-4 h-4 inline mr-1" />
                Spatial Simulation
              </Link>
            </div>
            <div className="flex items-center gap-4">
              <Button variant="ghost" size="sm">Sign In</Button>
              <Button size="sm" className="gradient-border">Get Started</Button>
            </div>
          </div>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="relative pt-24 pb-16 px-4 overflow-hidden">
        <div className="absolute inset-0 grid-bg opacity-30" />
        <div className="absolute inset-0 bg-gradient-to-b from-background via-background/95 to-background" />
        <div className="container mx-auto relative z-10">
          <div className="grid lg:grid-cols-2 gap-12 items-center">
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.6 }}
            >
              <Badge className="mb-6 glow">
                <Sparkles className="w-3 h-3 mr-1" />
                Next-Gen Robotics Learning Platform
              </Badge>
              <h1 className="text-4xl sm:text-5xl lg:text-6xl font-bold mb-6 leading-tight">
                Master{' '}
                <span className="gradient-text">Humanoid Robotics</span>
                <br />
                From 12 to Expert
              </h1>
              <p className="text-lg sm:text-xl text-muted-foreground mb-8 max-w-2xl">
                Build, program, and simulate advanced humanoid robots with our revolutionary platform.
                Access real robots, 3D simulations, and AI-powered learning paths.
              </p>
              <div className="flex flex-col sm:flex-row gap-4 mb-12">
                <Button size="lg" className="gradient-border text-base">
                  Start Learning Free
                  <ChevronRight className="w-4 h-4 ml-2" />
                </Button>
                <Button size="lg" variant="outline" className="glass">
                  <Play className="w-4 h-4 mr-2" />
                  Watch Demo
                </Button>
              </div>
              <div className="flex flex-wrap gap-6 text-sm text-muted-foreground">
                <div className="flex items-center gap-2">
                  <Users className="w-4 h-4 text-primary" />
                  <span>50,000+ Students</span>
                </div>
                <div className="flex items-center gap-2">
                  <Award className="w-4 h-4 text-primary" />
                  <span>Industry Certification</span>
                </div>
                <div className="flex items-center gap-2">
                  <Globe className="w-4 h-4 text-primary" />
                  <span>Global Community</span>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, scale: 0.9 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.6 }}
              className="relative"
            >
              <div className="relative rounded-2xl overflow-hidden gradient-border">
                <img
                  src="/images/hero-robot.png"
                  alt="Futuristic Humanoid Robot"
                  className="w-full h-auto animate-float"
                />
                <div className="absolute inset-0 bg-gradient-to-t from-background/80 to-transparent" />
              </div>
              {/* Floating Stats */}
              <div className="absolute -bottom-6 -left-6 glass rounded-xl p-4 shadow-xl animate-float">
                <div className="flex items-center gap-3">
                  <div className="w-12 h-12 rounded-lg bg-primary/20 flex items-center justify-center">
                    <Cpu className="w-6 h-6 text-primary" />
                  </div>
                  <div>
                    <div className="text-2xl font-bold">36+</div>
                    <div className="text-xs text-muted-foreground">Subjects</div>
                  </div>
                </div>
              </div>
              <div className="absolute -top-6 -right-6 glass rounded-xl p-4 shadow-xl animate-float" style={{ animationDelay: '1s' }}>
                <div className="flex items-center gap-3">
                  <div className="w-12 h-12 rounded-lg bg-accent/20 flex items-center justify-center">
                    <MonitorPlay className="w-6 h-6 text-accent" />
                  </div>
                  <div>
                    <div className="text-2xl font-bold">200+</div>
                    <div className="text-xs text-muted-foreground">Video Lessons</div>
                  </div>
                </div>
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section id="features" className="py-20 px-4">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl sm:text-4xl font-bold mb-4">
              Why Choose <span className="gradient-text">Our Platform</span>
            </h2>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              Revolutionary features that set new standards in robotics education
            </p>
          </motion.div>

          <motion.div
            variants={staggerContainer}
            initial="initial"
            whileInView="animate"
            viewport={{ once: true }}
            className="grid md:grid-cols-2 lg:grid-cols-3 gap-6"
          >
            {[
              {
                icon: <Box className="w-6 h-6" />,
                title: 'Real Robot Access',
                description: 'Program actual Unitree G1 and Poppy robots remotely with our digital twin technology'
              },
              {
                icon: <MonitorPlay className="w-6 h-6" />,
                title: '3D Simulations',
                description: 'MuJoCo-powered photorealistic simulations with real-time physics and haptics'
              },
              {
                icon: <Terminal className="w-6 h-6" />,
                title: 'Integrated Code Editor',
                description: 'Code in ROS2, Python, C++, and Rust with real-time compilation and execution'
              },
              {
                icon: <Brain className="w-6 h-6" />,
                title: 'AI-Powered Tutor',
                description: 'Personalized AI assistant that adapts to your learning style and pace'
              },
              {
                icon: <Headphones className="w-6 h-6" />,
                title: 'Voice Interaction',
                description: 'Full accessibility with voice commands and text-to-speech for all features'
              },
              {
                icon: <Zap className="w-6 h-6" />,
                title: 'Instant Feedback',
                description: 'Real-time code analysis, debugging, and performance optimization'
              }
            ].map((feature, index) => (
              <motion.div
                key={index}
                variants={fadeInUp}
                whileHover={{ scale: 1.02, transition: { duration: 0.2 } }}
              >
                <Card className="glass h-full border-border/50 hover:border-primary/50 transition-all duration-300">
                  <CardHeader>
                    <div className="w-12 h-12 rounded-lg bg-primary/20 flex items-center justify-center mb-4">
                      <div className="text-primary">{feature.icon}</div>
                    </div>
                    <CardTitle className="text-xl">{feature.title}</CardTitle>
                  </CardHeader>
                  <CardContent>
                    <CardDescription className="text-base">
                      {feature.description}
                    </CardDescription>
                  </CardContent>
                </Card>
              </motion.div>
            ))}
          </motion.div>
        </div>
      </section>

      {/* Learning Paths Section */}
      <section id="paths" className="py-20 px-4 bg-gradient-to-b from-background via-card/30 to-background">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl sm:text-4xl font-bold mb-4">
              Structured <span className="gradient-text">Learning Paths</span>
            </h2>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              From absolute beginner to robotics expert - a clear path for everyone
            </p>
          </motion.div>

          <div className="grid lg:grid-cols-3 gap-8">
            {[
              {
                level: 'Beginner',
                title: 'Robotics Fundamentals',
                pathId: 'path-beginner',
                duration: '3 Months',
                modules: ['Basic Electronics', 'Linux Essentials', 'Python for Robotics', 'ROS2 Basics'],
                icon: <BookOpen className="w-8 h-8" />,
                color: 'from-green-500/20 to-emerald-500/20'
              },
              {
                level: 'Intermediate',
                title: 'Humanoid Development',
                pathId: 'path-intermediate',
                duration: '6 Months',
                modules: ['Control Systems', 'Kinematics', 'Machine Learning', 'Computer Vision'],
                icon: <Layout className="w-8 h-8" />,
                color: 'from-blue-500/20 to-cyan-500/20'
              },
              {
                level: 'Advanced',
                title: 'AI & Robotics Mastery',
                pathId: 'path-advanced',
                duration: '9 Months',
                modules: ['Deep Learning', 'Reinforcement Learning', 'Advanced Control', 'Research Projects'],
                icon: <Rocket className="w-8 h-8" />,
                color: 'from-purple-500/20 to-pink-500/20'
              }
            ].map((path, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ delay: index * 0.1 }}
              >
                <Link href={`/learning-paths/${path.pathId}`}>
                  <Card className={`h-full glass border-border/50 hover:border-primary/50 transition-all duration-300`}>
                  <CardHeader className={`bg-gradient-to-r ${path.color} rounded-t-xl`}>
                    <div className="flex items-center justify-between mb-2">
                      <Badge variant="secondary">{path.level}</Badge>
                      <div className="text-primary">{path.icon}</div>
                    </div>
                    <CardTitle className="text-2xl">{path.title}</CardTitle>
                    <CardDescription>{path.duration}</CardDescription>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <ScrollArea className="h-48">
                      <ul className="space-y-3">
                        {path.modules.map((module, idx) => (
                          <li key={idx} className="flex items-center gap-2">
                            <ChevronRight className="w-4 h-4 text-primary flex-shrink-0" />
                            <span className="text-sm">{module}</span>
                          </li>
                        ))}
                      </ul>
                    </ScrollArea>
                    <Button className="w-full mt-4" variant="outline" onClick={() => window.location.href = `/learning-paths/${path.pathId}`}>
                      View Curriculum
                    </Button>
                  </CardContent>
                </Card>
                </Link>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Featured Courses Section */}
      <section id="courses" className="py-20 px-4">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl sm:text-4xl font-bold mb-4">
              Featured <span className="gradient-text">Courses</span>
            </h2>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              Hands-on projects with real robots and simulations
            </p>
          </motion.div>

          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
            {[
              {
                title: 'Unitree G1 Fundamentals',
                slug: 'unitree-g1-fundamentals',
                description: 'Master the G1 humanoid robot: walking control, navigation, and perception',
                image: '/images/unitree-g1.png',
                duration: '3 Days',
                level: 'Intermediate',
                students: '2,453'
              },
              {
                title: 'Build Your Poppy Robot',
                slug: 'python-robotics',
                description: 'Build and program your own humanoid robot from scratch',
                image: '/images/poppy-robot.png',
                duration: '4 Weeks',
                level: 'Beginner',
                students: '5,892'
              },
              {
                title: 'ROS2 & AI Integration',
                slug: 'ros2-ai-integration',
                description: 'Advanced ROS2 programming with AI algorithms for humanoid robots',
                image: '/images/ros2-diagram.png',
                duration: '6 Weeks',
                level: 'Advanced',
                students: '1,892'
              }
            ].map((course, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ delay: index * 0.1 }}
                whileHover={{ y: -8, transition: { duration: 0.2 } }}
              >
                <Link href={`/courses/${course.slug}`}>
                  <Card className="glass h-full border-border/50 hover:border-primary/50 transition-all duration-300 overflow-hidden group">
                  <div className="relative h-48 overflow-hidden">
                    <img
                      src={course.image}
                      alt={course.title}
                      className="w-full h-full object-cover group-hover:scale-110 transition-transform duration-300"
                    />
                    <div className="absolute inset-0 bg-gradient-to-t from-background to-transparent" />
                    <Badge className="absolute top-4 right-4" variant="secondary">
                      {course.level}
                    </Badge>
                  </div>
                  <CardHeader>
                    <CardTitle className="text-xl">{course.title}</CardTitle>
                    <CardDescription className="line-clamp-2">
                      {course.description}
                    </CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="flex items-center justify-between text-sm text-muted-foreground mb-4">
                      <span>⏱ {course.duration}</span>
                      <span>👥 {course.students} students</span>
                    </div>
                    <Button className="w-full" onClick={() => window.location.href = `/courses/${course.slug}`}>
                      Start Learning
                      <ChevronRight className="w-4 h-4 ml-2" />
                    </Button>
                  </CardContent>
                </Card>
                </Link>
              </motion.div>
            ))}
          </div>

          <div className="text-center mt-12">
            <Link href="/catalog">
              <Button size="lg" variant="outline" className="gradient-border">
                View All Courses
                <ChevronRight className="w-4 h-4 ml-2" />
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Platform Capabilities */}
      <section className="py-20 px-4 bg-gradient-to-b from-background via-card/30 to-background">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="grid lg:grid-cols-2 gap-12 items-center"
          >
            <div>
              <h2 className="text-3xl sm:text-4xl font-bold mb-6">
                <span className="gradient-text">Digital Twin</span> Technology
              </h2>
              <p className="text-lg text-muted-foreground mb-8">
                Our groundbreaking digital twin platform lets you program robots in simulation and deploy to real hardware seamlessly. Experience parallax synchronization between virtual and physical robots.
              </p>
              <div className="space-y-4">
                {[
                  'Real-time bidirectional synchronization',
                  'Photorealistic MuJoCo physics engine',
                  'Haptic feedback for immersive learning',
                  'Instant code deployment to real robots',
                  'Multi-robot fleet simulation'
                ].map((feature, index) => (
                  <div key={index} className="flex items-center gap-3">
                    <div className="w-6 h-6 rounded-full bg-primary/20 flex items-center justify-center flex-shrink-0">
                      <Shield className="w-3 h-3 text-primary" />
                    </div>
                    <span className="text-sm">{feature}</span>
                  </div>
                ))}
              </div>
            </div>
            <motion.div
              initial={{ opacity: 0, scale: 0.9 }}
              whileInView={{ opacity: 1, scale: 1 }}
              viewport={{ once: true }}
              className="relative"
            >
              <Card className="glass gradient-border overflow-hidden">
                <img
                  src="/images/simulation-interface.png"
                  alt="Simulation Interface"
                  className="w-full h-auto"
                />
              </Card>
            </motion.div>
          </motion.div>
        </div>
      </section>

      {/* Gamification Section */}
      <section className="py-20 px-4">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl sm:text-4xl font-bold mb-4">
              Learn. Build. <span className="gradient-text">Achieve.</span>
            </h2>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              Gamified learning with achievements, leaderboards, and rewards
            </p>
          </motion.div>

          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6">
            {[
              { icon: <Trophy className="w-8 h-8" />, title: 'Achievements', value: '100+' },
              { icon: <Award className="w-8 h-8" />, title: 'Certificates', value: '12' },
              { icon: <Target className="w-8 h-8" />, title: 'Milestones', value: '500+' },
              { icon: <MessageCircle className="w-8 h-8" />, title: 'Community', value: '50K+' }
            ].map((stat, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ delay: index * 0.1 }}
              >
                <Card className="glass text-center border-border/50">
                  <CardHeader>
                    <div className="mx-auto w-16 h-16 rounded-full bg-primary/20 flex items-center justify-center mb-4">
                      <div className="text-primary">{stat.icon}</div>
                    </div>
                    <CardTitle className="text-4xl font-bold gradient-text">
                      {stat.value}
                    </CardTitle>
                    <CardDescription className="text-base">
                      {stat.title}
                    </CardDescription>
                  </CardHeader>
                </Card>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-20 px-4">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
          >
            <Card className="glass gradient-border overflow-hidden">
              <div className="grid md:grid-cols-2 gap-8 p-8 md:p-12">
                <div>
                  <h2 className="text-3xl sm:text-4xl font-bold mb-6">
                    Start Your Robotics <span className="gradient-text">Journey Today</span>
                  </h2>
                  <p className="text-lg text-muted-foreground mb-8">
                    Join thousands of students building the future of humanoid robotics.
                    Get full access to courses, simulations, and real robots.
                  </p>
                  <div className="space-y-4">
                    <Button size="lg" className="w-full">
                      Get Started Free
                      <ChevronRight className="w-4 h-4 ml-2" />
                    </Button>
                    <Button size="lg" variant="outline" className="w-full glass">
                      Book a Demo
                    </Button>
                  </div>
                </div>
                <div className="flex items-center justify-center">
                  <img
                    src="/images/learning-dashboard.png"
                    alt="Learning Dashboard"
                    className="rounded-xl shadow-2xl max-w-full h-auto"
                  />
                </div>
              </div>
            </Card>
          </motion.div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-border/50 mt-auto">
        <div className="container mx-auto px-4 py-12">
          <div className="grid md:grid-cols-4 gap-8 mb-8">
            <div>
              <div className="flex items-center gap-2 mb-4">
                <Cpu className="h-6 w-6 text-primary" />
                <span className="font-bold gradient-text">ROBOTICS MASTERY</span>
              </div>
              <p className="text-sm text-muted-foreground">
                The future of humanoid robotics education. Learn, build, and innovate with next-gen tools.
              </p>
            </div>
            <div>
              <h3 className="font-semibold mb-4">Platform</h3>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li><Link href="#" className="hover:text-foreground transition-colors">Courses</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Simulations</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Real Robots</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Pricing</Link></li>
              </ul>
            </div>
            <div>
              <h3 className="font-semibold mb-4">Resources</h3>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li><Link href="#" className="hover:text-foreground transition-colors">Documentation</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">API Reference</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Community</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Blog</Link></li>
              </ul>
            </div>
            <div>
              <h3 className="font-semibold mb-4">Company</h3>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li><Link href="#" className="hover:text-foreground transition-colors">About Us</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Careers</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Contact</Link></li>
                <li><Link href="#" className="hover:text-foreground transition-colors">Privacy</Link></li>
              </ul>
            </div>
          </div>
          <div className="border-t border-border/50 pt-8 flex flex-col md:flex-row justify-between items-center gap-4">
            <p className="text-sm text-muted-foreground">
              © 2025 Robotics Mastery. All rights reserved.
            </p>
            <div className="flex items-center gap-4 text-sm text-muted-foreground">
              <Link href="#" className="hover:text-foreground transition-colors">Terms</Link>
              <Link href="#" className="hover:text-foreground transition-colors">Privacy</Link>
              <Link href="#" className="hover:text-foreground transition-colors">Cookies</Link>
            </div>
          </div>
        </div>
      </footer>
    </div>
  )
}
