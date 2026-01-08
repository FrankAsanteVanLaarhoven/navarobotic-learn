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
import { UtilityMenu } from '@/components/utility-menu'
import { AnimatedLogo } from '@/components/AnimatedLogo'
import { MissionImpossibleText } from '@/components/MissionImpossibleText'
import { ThemeCustomizer } from '@/components/ThemeCustomizer'
import { SplineScene } from '@/components/ui/spline'
import { Spotlight } from '@/components/ui/spotlight'
import { RIcon } from '@/components/RIcon'
import { Navbar } from '@/components/Navbar'
import { MediaCenter } from '@/components/MediaCenter'
import { DraggableEnrollButton } from '@/components/DraggableEnrollButton'

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
      <Navbar />

      {/* Hero Section */}
      <section className="relative pt-32 pb-16 px-4 overflow-hidden">
        <div className="absolute inset-0 grid-bg opacity-30" />
        <div className="absolute inset-0 bg-gradient-to-b from-background via-background/95 to-background" />
        <div className="container mx-auto relative z-10">
          <div className="grid lg:grid-cols-2 gap-12 items-center">
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ duration: 0.6 }}
            >
              <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-6 leading-tight">
                Learn robotics by building.
              </MissionImpossibleText>
              <MissionImpossibleText variant="body" glowIntensity="medium" className="mb-8 max-w-2xl">
                Courses, 3D simulation, and real-robot labs in one platform.
              </MissionImpossibleText>
              <div className="flex flex-col sm:flex-row gap-4 mb-12">
                <Button size="lg" className="text-base">
                  <MissionImpossibleText variant="label" glowIntensity="medium">
                    Start learning
                  </MissionImpossibleText>
                  <ChevronRight className="w-4 h-4 ml-2" style={{
                    filter: 'drop-shadow(0 0 4px var(--mission-halo-color, rgba(255, 107, 53, 0.6)))'
                  }} />
                </Button>
                <Button size="lg" variant="outline" className="glass">
                  <Play className="w-4 h-4 mr-2" style={{
                    filter: 'drop-shadow(0 0 4px var(--mission-halo-color, rgba(255, 107, 53, 0.6)))'
                  }} />
                  <MissionImpossibleText variant="label" glowIntensity="medium">
                    Try the simulator
                  </MissionImpossibleText>
                </Button>
              </div>
              <div className="flex flex-wrap gap-6 text-sm text-muted-foreground">
                <div className="flex items-center gap-2">
                  <Users className="w-4 h-4 text-primary" />
                  <MissionImpossibleText variant="label" glowIntensity="low">50,000+ Students</MissionImpossibleText>
                </div>
                <div className="flex items-center gap-2">
                  <Award className="w-4 h-4 text-primary" />
                  <MissionImpossibleText variant="label" glowIntensity="low">Industry Certification</MissionImpossibleText>
                </div>
                <div className="flex items-center gap-2">
                  <Globe className="w-4 h-4 text-primary" />
                  <MissionImpossibleText variant="label" glowIntensity="low">Global Community</MissionImpossibleText>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, scale: 0.9 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.6 }}
              className="relative"
            >
              <Card className="w-full h-[500px] bg-black/[0.96] relative overflow-hidden rounded-2xl border-border/50">
                <Spotlight
                  className="-top-40 left-0 md:left-60 md:-top-20"
                  fill="white"
                />
                
                {/* Spline Scene - Full width */}
                <div className="relative w-full h-full">
                  <SplineScene 
                    scene="https://prod.spline.design/kZDDjO5HuC9GJUM2/scene.splinecode"
                    className="w-full h-full"
                  />
                </div>
              </Card>
              {/* Floating Stats */}
              <div className="absolute -bottom-6 -left-6 glass rounded-xl p-4 shadow-xl animate-float">
                <div className="flex items-center gap-3">
                  <RIcon size={48} />
                  <div>
                    <MissionImpossibleText variant="body" glowIntensity="medium" className="text-2xl font-bold">
                      36+
                    </MissionImpossibleText>
                    <MissionImpossibleText variant="small" glowIntensity="low" className="text-xs">
                      Subjects
                    </MissionImpossibleText>
                  </div>
                </div>
              </div>
              <div className="absolute -top-6 -right-6 glass rounded-xl p-4 shadow-xl animate-float" style={{ animationDelay: '1s' }}>
                <div className="flex items-center gap-3">
                  <div className="w-12 h-12 rounded-lg bg-accent/20 flex items-center justify-center" style={{
                    boxShadow: '0 0 10px rgba(255, 209, 102, 0.3)',
                    border: '1px solid rgba(255, 209, 102, 0.2)'
                  }}>
                    <MonitorPlay className="w-6 h-6 text-accent" style={{
                      filter: 'drop-shadow(0 0 4px rgba(255, 209, 102, 0.6)) drop-shadow(0 0 8px rgba(255, 209, 102, 0.4))'
                    }} />
                  </div>
                  <div>
                    <MissionImpossibleText variant="body" glowIntensity="medium" className="text-2xl font-bold">
                      200+
                    </MissionImpossibleText>
                    <MissionImpossibleText variant="small" glowIntensity="low" className="text-xs">
                      Video Lessons
                    </MissionImpossibleText>
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
            <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-4">
              Build real robots, learn by doing
            </MissionImpossibleText>
            <MissionImpossibleText variant="body" glowIntensity="medium" className="max-w-2xl mx-auto">
              Everything you need to go from beginner to deployment
            </MissionImpossibleText>
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
                title: 'Hands-on learning',
                description: 'Short labs that ship to real robots.'
              },
              {
                icon: <MonitorPlay className="w-6 h-6" />,
                title: '3D simulation',
                description: 'Browser-based ROS/physics; fail fast, fix faster.'
              },
              {
                icon: <Video className="w-6 h-6" />,
                title: 'AI video & feedback',
                description: 'Auto-generated walkthroughs, instant checks.'
              },
              {
                icon: <Target className="w-6 h-6" />,
                title: 'Career tracks',
                description: 'From fundamentals to deployment.'
              },
              {
                icon: <Users className="w-6 h-6" />,
                title: 'For campuses & teams',
                description: 'Admin, analytics, SSO.'
              },
              {
                icon: <Code className="w-6 h-6" />,
                title: 'Real-robot labs',
                description: 'Deploy code directly to physical hardware.'
              }
            ].map((feature, index) => {
              const featureImages = [
                '/images/feature-1.png',
                '/images/feature-2.png',
                '/images/feature-3.png',
                '/images/feature-4.png',
                '/images/feature-5.png'
              ]
              return (
                <motion.div
                  key={index}
                  variants={fadeInUp}
                  whileHover={{ scale: 1.02, transition: { duration: 0.2 } }}
                >
                  <Card className="glass h-full border-border/50 hover:border-primary/50 transition-all duration-300 overflow-hidden">
                    <CardHeader>
                      <div className="w-12 h-12 rounded-lg bg-primary/20 flex items-center justify-center mb-4">
                        <div className="text-primary">{feature.icon}</div>
                      </div>
                      <MissionImpossibleText variant="label" glowIntensity="medium" className="text-xl">
                        {feature.title}
                      </MissionImpossibleText>
                    </CardHeader>
                    <CardContent>
                      <MissionImpossibleText variant="small" glowIntensity="low" className="mb-4">
                        {feature.description}
                      </MissionImpossibleText>
                      {index < featureImages.length && (
                        <div className="mt-4 rounded-lg overflow-hidden">
                          <img
                            src={featureImages[index]}
                            alt={feature.title}
                            className="w-full h-32 object-cover opacity-60 hover:opacity-100 transition-opacity"
                          />
                        </div>
                      )}
                    </CardContent>
                  </Card>
                </motion.div>
              )
            })}
          </motion.div>
        </div>
      </section>

      {/* Media Center Section */}
      <MediaCenter />

      {/* Learning Paths Section */}
      <section id="paths" className="py-20 px-4 bg-gradient-to-b from-background via-card/30 to-background">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-16"
          >
            <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-4">
              Structured Learning Paths
            </MissionImpossibleText>
            <MissionImpossibleText variant="body" glowIntensity="medium" className="max-w-2xl mx-auto">
              From absolute beginner to robotics expert - a clear path for everyone
            </MissionImpossibleText>
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
                    <MissionImpossibleText variant="body" glowIntensity="medium" className="text-2xl">
                      {path.title}
                    </MissionImpossibleText>
                    <MissionImpossibleText variant="small" glowIntensity="low">
                      {path.duration}
                    </MissionImpossibleText>
                  </CardHeader>
                  <CardContent className="pt-6">
                    <ScrollArea className="h-48">
                      <ul className="space-y-3">
                        {path.modules.map((module, idx) => (
                          <li key={idx} className="flex items-center gap-2">
                            <ChevronRight className="w-4 h-4 text-primary flex-shrink-0" />
                            <MissionImpossibleText variant="small" glowIntensity="low">
                              {module}
                            </MissionImpossibleText>
                          </li>
                        ))}
                      </ul>
                    </ScrollArea>
                    <Button className="w-full mt-4" variant="outline" onClick={() => window.location.href = `/learning-paths/${path.pathId}`}>
                      <MissionImpossibleText variant="small" glowIntensity="low">View Curriculum</MissionImpossibleText>
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
            <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-4">
              Featured Courses
            </MissionImpossibleText>
            <MissionImpossibleText variant="body" glowIntensity="medium" className="max-w-2xl mx-auto">
              Hands-on projects with real robots and simulations
            </MissionImpossibleText>
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
              <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-6" withFrame>
                Digital Twin Technology
              </MissionImpossibleText>
              <MissionImpossibleText variant="body" glowIntensity="medium" className="mb-8">
                Our groundbreaking digital twin platform lets you program robots in simulation and deploy to real hardware seamlessly. Experience parallax synchronization between virtual and physical robots.
              </MissionImpossibleText>
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
                    <MissionImpossibleText variant="small" glowIntensity="low">
                      {feature}
                    </MissionImpossibleText>
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
            <MissionImpossibleText variant="heading" glowIntensity="high" className="mb-4">
              Learn. Build. Achieve.
            </MissionImpossibleText>
            <MissionImpossibleText variant="body" glowIntensity="medium" className="max-w-2xl mx-auto">
              Gamified learning with achievements, leaderboards, and rewards
            </MissionImpossibleText>
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
                    <MissionImpossibleText variant="heading" glowIntensity="high" className="text-4xl font-bold">
                      {stat.value}
                    </MissionImpossibleText>
                    <MissionImpossibleText variant="body" glowIntensity="medium" className="text-base">
                      {stat.title}
                    </MissionImpossibleText>
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
            className="relative"
          >
            <div className="relative w-full">
              {/* CTA Image with faded edges - zero opacity at edges */}
              <div className="relative w-full">
                {/* Image with radial mask for edge fading */}
                <div 
                  className="w-full relative"
                  style={{
                    maskImage: 'radial-gradient(ellipse 90% 90% at center, black 70%, transparent 100%)',
                    WebkitMaskImage: 'radial-gradient(ellipse 90% 90% at center, black 70%, transparent 100%)',
                  }}
                >
                  <img
                    src="/images/image_f8e2a28e-c59e-4b35-a756-d98f91270dd3.png"
                    alt="Start Your Robotics Journey Today"
                    className="w-full h-auto"
                  />
                </div>
                
                {/* Additional gradient overlays for smoother edge blend */}
                <div className="absolute inset-0 pointer-events-none z-10">
                  {/* Top fade - zero opacity at top */}
                  <div className="absolute top-0 left-0 right-0 h-40 bg-gradient-to-b from-background via-background/30 to-transparent" />
                  {/* Bottom fade - zero opacity at bottom */}
                  <div className="absolute bottom-0 left-0 right-0 h-40 bg-gradient-to-t from-background via-background/30 to-transparent" />
                  {/* Left fade - zero opacity at left */}
                  <div className="absolute top-0 bottom-0 left-0 w-40 bg-gradient-to-r from-background via-background/30 to-transparent" />
                  {/* Right fade - zero opacity at right */}
                  <div className="absolute top-0 bottom-0 right-0 w-40 bg-gradient-to-l from-background via-background/30 to-transparent" />
                </div>
                
                {/* Draggable Floating Enroll Now Button */}
                <DraggableEnrollButton />
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-border/50 mt-auto">
        <div className="container mx-auto px-4 py-12">
          <div className="grid md:grid-cols-4 gap-8 mb-8">
            <div>
              <div className="flex items-center gap-2 mb-4">
                <AnimatedLogo size="sm" />
              </div>
              <MissionImpossibleText variant="small" glowIntensity="low">
                The future of humanoid robotics education. Learn, build, and innovate with next-gen tools.
              </MissionImpossibleText>
            </div>
            <div>
              <MissionImpossibleText variant="label" glowIntensity="medium" className="mb-4 font-semibold">Platform</MissionImpossibleText>
              <ul className="space-y-2">
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Courses</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Simulations</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Real Robots</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Pricing</MissionImpossibleText></Link></li>
              </ul>
            </div>
            <div>
              <MissionImpossibleText variant="label" glowIntensity="medium" className="mb-4 font-semibold">Resources</MissionImpossibleText>
              <ul className="space-y-2">
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Documentation</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">API Reference</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Community</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Blog</MissionImpossibleText></Link></li>
              </ul>
            </div>
            <div>
              <MissionImpossibleText variant="label" glowIntensity="medium" className="mb-4 font-semibold">Company</MissionImpossibleText>
              <ul className="space-y-2">
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">About Us</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Careers</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Contact</MissionImpossibleText></Link></li>
                <li><Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Privacy</MissionImpossibleText></Link></li>
              </ul>
            </div>
          </div>
          <div className="border-t border-border/50 pt-8 flex flex-col md:flex-row justify-between items-center gap-4">
            <MissionImpossibleText variant="small" glowIntensity="low">
              © 2026 Rovyn. All rights reserved.
            </MissionImpossibleText>
            <div className="flex items-center gap-4">
              <Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Terms</MissionImpossibleText></Link>
              <Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Privacy</MissionImpossibleText></Link>
              <Link href="#"><MissionImpossibleText variant="small" glowIntensity="low">Cookies</MissionImpossibleText></Link>
            </div>
          </div>
        </div>
      </footer>
    </div>
  )
}
