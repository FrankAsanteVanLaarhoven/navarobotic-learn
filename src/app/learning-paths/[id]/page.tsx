'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Progress } from '@/components/ui/progress'
import {
  Clock,
  BookOpen,
  ChevronRight,
  Play,
  Target,
  Award,
  CheckCircle2,
  Lock,
  ArrowRight
} from 'lucide-react'
import { useState, useEffect, use } from 'react'
import Link from 'next/link'

interface Course {
  id: string
  slug: string
  title: string
  description: string
  level: string
  duration: string
  thumbnail?: string
  totalStudents: number
  rating: number
  order?: number
}

interface LearningPath {
  id: string
  title: string
  description: string
  level: string
  duration: string
  courses: Course[]
}

export default function LearningPathPage({ params }: { params: Promise<{ id: string }> }) {
  const { id } = use(params)
  const [path, setPath] = useState<LearningPath | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    fetchPath()
  }, [id])

  const fetchPath = async () => {
    try {
      const response = await fetch('/api/learning-paths')
      const paths = await response.json()
      const found = paths.find((p: LearningPath) => p.id === id)
      if (found) {
        setPath(found)
      }
    } catch (error) {
      console.error('Error fetching learning path:', error)
    } finally {
      setLoading(false)
    }
  }

  const getLevelColor = (level: string) => {
    switch (level.toLowerCase()) {
      case 'beginner':
        return 'from-green-500/20 to-emerald-500/20 text-green-500'
      case 'intermediate':
        return 'from-blue-500/20 to-cyan-500/20 text-blue-500'
      case 'advanced':
        return 'from-purple-500/20 to-pink-500/20 text-purple-500'
      default:
        return 'from-gray-500/20 to-slate-500/20 text-gray-500'
    }
  }

  const getBadgeColor = (level: string) => {
    switch (level.toLowerCase()) {
      case 'beginner':
        return 'bg-green-500/10 text-green-500 hover:bg-green-500/20'
      case 'intermediate':
        return 'bg-blue-500/10 text-blue-500 hover:bg-blue-500/20'
      case 'advanced':
        return 'bg-purple-500/10 text-purple-500 hover:bg-purple-500/20'
      default:
        return 'bg-gray-500/10 text-gray-500 hover:bg-gray-500/20'
    }
  }

  if (loading) {
    return (
      <div className="min-h-screen flex items-center justify-center pt-16">
        <div className="text-center">
          <div className="inline-block animate-pulse-slow">
            <BookOpen className="w-16 h-16 text-primary animate-pulse mx-auto mb-4" />
          </div>
          <p className="text-lg text-muted-foreground">Loading learning path...</p>
        </div>
      </div>
    )
  }

  if (!path) {
    return (
      <div className="min-h-screen flex items-center justify-center pt-16">
        <Card className="max-w-md w-full">
          <CardHeader>
            <CardTitle>Learning Path Not Found</CardTitle>
            <CardDescription>
              The learning path you're looking for doesn't exist or has been removed.
            </CardDescription>
          </CardHeader>
          <CardContent>
            <Link href="/catalog">
              <Button className="w-full">
                Back to Catalog
                <ChevronRight className="w-4 h-4 ml-2" />
              </Button>
            </Link>
          </CardContent>
        </Card>
      </div>
    )
  }

  return (
    <div className="min-h-screen">
      {/* Header */}
      <section className="pt-24 pb-12 px-4 border-b border-border/50 bg-gradient-to-b from-background via-card/30 to-background">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            className="max-w-4xl mx-auto text-center"
          >
            <Link href="/catalog" className="text-sm text-muted-foreground hover:text-foreground transition-colors mb-4 inline-block">
              ← Back to Catalog
            </Link>

            <Badge className={`capitalize ${getBadgeColor(path.level)} mb-6 px-4 py-2`}>
              {path.level} Level
            </Badge>

            <h1 className="text-4xl sm:text-5xl font-bold mb-6">
              {path.title}
            </h1>

            <p className="text-xl text-muted-foreground max-w-2xl mx-auto mb-8">
              {path.description}
            </p>

            <div className="flex flex-wrap justify-center gap-8 text-sm">
              <div className="flex items-center gap-2">
                <Clock className="w-5 h-5 text-primary" />
                <span className="font-semibold">{path.duration}</span>
              </div>
              <div className="flex items-center gap-2">
                <BookOpen className="w-5 h-5 text-primary" />
                <span className="font-semibold">{path.courses.length} Courses</span>
              </div>
              <div className="flex items-center gap-2">
                <Award className="w-5 h-5 text-primary" />
                <span className="font-semibold">Certificate</span>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Progress Overview */}
      <section className="py-8 px-4 border-b border-border/50">
        <div className="container mx-auto max-w-4xl">
          <Card className="glass">
            <CardContent className="p-6">
              <div className="grid md:grid-cols-3 gap-6">
                <div className="text-center">
                  <div className="text-3xl font-bold gradient-text mb-2">0%</div>
                  <div className="text-sm text-muted-foreground">Progress</div>
                </div>
                <div className="text-center">
                  <div className="text-3xl font-bold gradient-text mb-2">0</div>
                  <div className="text-sm text-muted-foreground">Courses Completed</div>
                </div>
                <div className="text-center">
                  <div className="text-3xl font-bold gradient-text mb-2">0h</div>
                  <div className="text-sm text-muted-foreground">Time Spent</div>
                </div>
              </div>

              <div className="mt-6">
                <div className="flex items-center justify-between text-sm mb-2">
                  <span className="text-muted-foreground">Overall Progress</span>
                  <span className="font-semibold">0%</span>
                </div>
                <Progress value={0} className="h-2" />
              </div>
            </CardContent>
          </Card>
        </div>
      </section>

      {/* Courses List */}
      <section className="py-12 px-4">
        <div className="container mx-auto max-w-4xl">
          <h2 className="text-2xl font-bold mb-8 flex items-center gap-2">
            <Target className="w-6 h-6 text-primary" />
            Course Roadmap
          </h2>

          <div className="space-y-6">
            {path.courses.map((course, index) => (
              <motion.div
                key={course.id}
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: index * 0.1 }}
              >
                <Link href={`/courses/${course.slug}`}>
                  <Card className="glass border-border/50 hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                    <CardContent className="p-6">
                      <div className="flex items-start gap-6">
                        {/* Step Indicator */}
                        <div className="flex-shrink-0">
                          <div className={`w-12 h-12 rounded-xl bg-gradient-to-br ${getLevelColor(course.level)} flex items-center justify-center`}>
                            <span className="text-xl font-bold">{index + 1}</span>
                          </div>
                        </div>

                        {/* Course Info */}
                        <div className="flex-1 min-w-0">
                          <div className="flex items-start justify-between gap-4 mb-2">
                            <div className="flex-1">
                              <div className="flex items-center gap-2 mb-1">
                                <h3 className="text-xl font-semibold group-hover:text-primary transition-colors">
                                  {course.title}
                                </h3>
                                <Badge className={`capitalize ${getBadgeColor(course.level)}`}>
                                  {course.level}
                                </Badge>
                              </div>
                              <p className="text-muted-foreground line-clamp-2">
                                {course.description}
                              </p>
                            </div>
                          </div>

                          <div className="flex flex-wrap items-center gap-4 text-sm text-muted-foreground mt-3">
                            <div className="flex items-center gap-1">
                              <Clock className="w-4 h-4" />
                              <span>{course.duration}</span>
                            </div>
                            <div className="flex items-center gap-1">
                              <BookOpen className="w-4 h-4" />
                              <span>{course.totalStudents} students</span>
                            </div>
                          </div>

                          <div className="mt-4 flex items-center justify-between">
                            <Button variant="outline" size="sm">
                              View Course
                              <ChevronRight className="w-4 h-4 ml-1" />
                            </Button>
                          </div>
                        </div>

                        {/* Status */}
                        <div className="flex-shrink-0">
                          <Lock className="w-5 h-5 text-muted-foreground" />
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                </Link>

                {/* Connector Line */}
                {index < path.courses.length - 1 && (
                  <div className="flex justify-center ml-6">
                    <div className="w-0.5 h-6 bg-border" />
                  </div>
                )}
              </motion.div>
            ))}
          </div>

          {/* CTA */}
          <div className="mt-12 text-center">
            <Card className="glass gradient-border">
              <CardContent className="p-8">
                <Award className="w-12 h-12 text-primary mx-auto mb-4" />
                <h3 className="text-2xl font-bold mb-4">
                  Complete This Path, Earn a Certificate
                </h3>
                <p className="text-muted-foreground mb-6 max-w-md mx-auto">
                  Master all courses in this learning path to receive an official certificate of completion
                </p>
                <Button size="lg" className="gradient-border">
                  Start First Course
                  <ArrowRight className="w-5 h-5 ml-2" />
                </Button>
              </CardContent>
            </Card>
          </div>
        </div>
      </section>
    </div>
  )
}
