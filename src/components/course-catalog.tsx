'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Accordion, AccordionContent, AccordionItem, AccordionTrigger } from '@/components/ui/accordion'
import {
  BookOpen,
  Clock,
  Users,
  ChevronRight,
  Play,
  CheckCircle2,
  Lock
} from 'lucide-react'
import { useState, useEffect } from 'react'

interface LearningPath {
  id: string
  title: string
  description: string
  level: string
  duration: string
  courses: Course[]
}

interface Course {
  id: string
  title: string
  slug: string
  description: string
  level: string
  duration: string
  thumbnail?: string
  totalStudents: number
  rating: number
}

export default function CourseCatalog() {
  const [learningPaths, setLearningPaths] = useState<LearningPath[]>([])
  const [loading, setLoading] = useState(true)
  const [selectedPath, setSelectedPath] = useState<string | null>(null)

  useEffect(() => {
    fetchLearningPaths()
  }, [])

  const fetchLearningPaths = async () => {
    try {
      const response = await fetch('/api/learning-paths')
      const data = await response.json()
      setLearningPaths(data)
    } catch (error) {
      console.error('Error fetching learning paths:', error)
    } finally {
      setLoading(false)
    }
  }

  const getPathColor = (level: string) => {
    switch (level.toLowerCase()) {
      case 'beginner':
        return 'from-green-500/20 to-emerald-500/20'
      case 'intermediate':
        return 'from-blue-500/20 to-cyan-500/20'
      case 'advanced':
        return 'from-purple-500/20 to-pink-500/20'
      default:
        return 'from-gray-500/20 to-slate-500/20'
    }
  }

  const getLevelIcon = (level: string) => {
    switch (level.toLowerCase()) {
      case 'beginner':
        return <BookOpen className="w-5 h-5" />
      case 'intermediate':
        return <Play className="w-5 h-5" />
      case 'advanced':
        return <CheckCircle2 className="w-5 h-5" />
      default:
        return <BookOpen className="w-5 h-5" />
    }
  }

  if (loading) {
    return (
      <section className="py-20 px-4">
        <div className="container mx-auto">
          <div className="text-center">
            <div className="inline-block animate-pulse-slow">
              <div className="w-16 h-16 rounded-full bg-primary/20 flex items-center justify-center mx-auto mb-4">
                <BookOpen className="w-8 h-8 text-primary animate-pulse" />
              </div>
            </div>
            <p className="text-lg text-muted-foreground">Loading courses...</p>
          </div>
        </div>
      </section>
    )
  }

  return (
    <section id="courses" className="py-20 px-4">
      <div className="container mx-auto">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="text-center mb-16"
        >
          <h2 className="text-3xl sm:text-4xl font-bold mb-4">
            Course <span className="gradient-text">Catalog</span>
          </h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            Explore our comprehensive learning paths designed for every skill level
          </p>
        </motion.div>

        <div className="grid lg:grid-cols-3 gap-8">
          {learningPaths.map((path, index) => (
            <motion.div
              key={path.id}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: index * 0.1 }}
            >
              <Card className="glass h-full border-border/50 hover:border-primary/50 transition-all duration-300">
                <CardHeader className={`bg-gradient-to-r ${getPathColor(path.level)} rounded-t-xl`}>
                  <div className="flex items-center justify-between mb-2">
                    <Badge variant="secondary" className="capitalize">
                      {path.level}
                    </Badge>
                    <div className="text-primary">
                      {getLevelIcon(path.level)}
                    </div>
                  </div>
                  <CardTitle className="text-2xl">{path.title}</CardTitle>
                  <CardDescription>{path.duration}</CardDescription>
                </CardHeader>
                <CardContent className="pt-6">
                  <p className="text-sm text-muted-foreground mb-4">
                    {path.description}
                  </p>

                  <Accordion type="single" collapsible className="w-full">
                    <AccordionItem value="courses">
                      <AccordionTrigger className="text-sm">
                        Courses ({path.courses.length})
                      </AccordionTrigger>
                      <AccordionContent>
                        <ScrollArea className="h-64 pr-4">
                          <div className="space-y-3 pt-2">
                            {path.courses.map((course) => (
                              <motion.div
                                key={course.id}
                                whileHover={{ x: 4 }}
                                className="group"
                              >
                                <Card className="glass p-4 hover:border-primary/50 transition-all duration-300 cursor-pointer">
                                  <div className="flex items-start justify-between gap-3">
                                    <div className="flex-1 min-w-0">
                                      <h4 className="font-semibold text-sm mb-1 group-hover:text-primary transition-colors">
                                        {course.title}
                                      </h4>
                                      <p className="text-xs text-muted-foreground line-clamp-2">
                                        {course.description}
                                      </p>
                                      <div className="flex items-center gap-4 mt-2 text-xs text-muted-foreground">
                                        <div className="flex items-center gap-1">
                                          <Clock className="w-3 h-3" />
                                          <span>{course.duration}</span>
                                        </div>
                                        <div className="flex items-center gap-1">
                                          <Users className="w-3 h-3" />
                                          <span>{course.totalStudents} students</span>
                                        </div>
                                      </div>
                                    </div>
                                    <Button size="sm" variant="ghost" className="flex-shrink-0">
                                      <ChevronRight className="w-4 h-4" />
                                    </Button>
                                  </div>
                                </Card>
                              </motion.div>
                            ))}
                          </div>
                        </ScrollArea>
                      </AccordionContent>
                    </AccordionItem>
                  </Accordion>

                  <Button className="w-full mt-4" variant="outline">
                    View Full Curriculum
                  </Button>
                </CardContent>
              </Card>
            </motion.div>
          ))}
        </div>

        <div className="text-center mt-12">
          <Button size="lg" variant="outline" className="gradient-border">
            Browse All Courses
            <ChevronRight className="w-4 h-4 ml-2" />
          </Button>
        </div>
      </div>
    </section>
  )
}
