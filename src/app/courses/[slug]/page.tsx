'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Separator } from '@/components/ui/separator'
import { Avatar, AvatarFallback, AvatarImage } from '@/components/ui/avatar'
import {
  Clock,
  Users,
  Star,
  BookOpen,
  Play,
  ChevronRight,
  CheckCircle2,
  Lock,
  Video,
  Code,
  Terminal
} from 'lucide-react'
import { useState, useEffect, use } from 'react'
import Link from 'next/link'

interface Lesson {
  id: string
  title: string
  content: string
  videoUrl?: string
  videoDuration?: number
  order: number
  language?: string
  codeTemplate?: string
}

interface Module {
  id: string
  title: string
  description?: string
  order: number
  lessons: Lesson[]
}

interface Course {
  id: string
  title: string
  slug: string
  description: string
  level: string
  duration: string
  thumbnail?: string
  videoPreview?: string
  totalStudents: number
  rating: number
  modules: Module[]
  learningPath?: {
    title: string
  }
}

export default function CoursePage({ params }: { params: Promise<{ slug: string }> }) {
  const { slug } = use(params)
  const [course, setCourse] = useState<Course | null>(null)
  const [loading, setLoading] = useState(true)
  const [selectedLesson, setSelectedLesson] = useState<Lesson | null>(null)
  const [currentModuleIndex, setCurrentModuleIndex] = useState(0)
  const [currentLessonIndex, setCurrentLessonIndex] = useState(0)

  useEffect(() => {
    fetchCourse()
  }, [slug])

  const fetchCourse = async () => {
    try {
      const response = await fetch(`/api/courses/slug/${slug}`)
      if (response.ok) {
        const data = await response.json()
        setCourse(data)
        // Select first lesson if available
        if (data.modules?.[0]?.lessons?.[0]) {
          setSelectedLesson(data.modules[0].lessons[0])
        }
      } else {
        // Try by ID
        const allCourses = await fetch('/api/courses').then(r => r.json())
        const found = allCourses.find((c: Course) => c.slug === slug)
        if (found) {
          setCourse(found)
          if (found.modules?.[0]?.lessons?.[0]) {
            setSelectedLesson(found.modules[0].lessons[0])
          }
        }
      }
    } catch (error) {
      console.error('Error fetching course:', error)
    } finally {
      setLoading(false)
    }
  }

  const selectLesson = (moduleIndex: number, lessonIndex: number) => {
    const courseModule = course?.modules[moduleIndex]
    const lesson = courseModule?.lessons[lessonIndex]
    if (lesson) {
      setSelectedLesson(lesson)
      setCurrentModuleIndex(moduleIndex)
      setCurrentLessonIndex(lessonIndex)
    }
  }

  const getLevelColor = (level: string) => {
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

  const formatDuration = (seconds?: number) => {
    if (!seconds) return 'N/A'
    const hours = Math.floor(seconds / 3600)
    const minutes = Math.floor((seconds % 3600) / 60)
    if (hours > 0) return `${hours}h ${minutes}m`
    return `${minutes}m`
  }

  if (loading) {
    return (
      <div className="min-h-screen flex items-center justify-center pt-16">
        <div className="text-center">
          <div className="inline-block animate-pulse-slow">
            <BookOpen className="w-16 h-16 text-primary animate-pulse mx-auto mb-4" />
          </div>
          <p className="text-lg text-muted-foreground">Loading course...</p>
        </div>
      </div>
    )
  }

  if (!course) {
    return (
      <div className="min-h-screen flex items-center justify-center pt-16">
        <Card className="max-w-md w-full">
          <CardHeader>
            <CardTitle>Course Not Found</CardTitle>
            <CardDescription>
              The course you're looking for doesn't exist or has been removed.
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
      {/* Course Header */}
      <section className="pt-24 pb-8 px-4 border-b border-border/50">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
          >
            <Link href="/catalog" className="text-sm text-muted-foreground hover:text-foreground transition-colors mb-4 inline-block">
              ← Back to Catalog
            </Link>

            <div className="grid lg:grid-cols-3 gap-8">
              <div className="lg:col-span-2">
                <Badge className={`capitalize ${getLevelColor(course.level)} mb-4`}>
                  {course.level}
                </Badge>
                <h1 className="text-3xl sm:text-4xl font-bold mb-4">
                  {course.title}
                </h1>
                <p className="text-lg text-muted-foreground mb-6">
                  {course.description}
                </p>

                <div className="flex flex-wrap gap-6 text-sm text-muted-foreground mb-6">
                  <div className="flex items-center gap-2">
                    <Clock className="w-4 h-4 text-primary" />
                    <span>{course.duration}</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Users className="w-4 h-4 text-primary" />
                    <span>{course.totalStudents} students</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <Star className="w-4 h-4 text-primary fill-yellow-500" />
                    <span className="font-semibold">{course.rating} rating</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <BookOpen className="w-4 h-4 text-primary" />
                    <span>{course.modules.length} modules</span>
                  </div>
                </div>
              </div>

              <div className="lg:col-span-1">
                <Card className="glass">
                  <CardHeader>
                    <CardTitle>Course Info</CardTitle>
                  </CardHeader>
                  <CardContent className="space-y-4">
                    <div className="flex items-center gap-3">
                      <Avatar className="h-12 w-12">
                        <AvatarImage src="/images/unitree-g1.png" />
                        <AvatarFallback>IC</AvatarFallback>
                      </Avatar>
                      <div>
                        <p className="font-semibold text-sm">Instructor</p>
                        <p className="text-xs text-muted-foreground">Robotics Mastery Team</p>
                      </div>
                    </div>

                    <Separator />

                    <div className="space-y-2">
                      <div className="flex justify-between text-sm">
                        <span className="text-muted-foreground">Level</span>
                        <span className="capitalize font-semibold">{course.level}</span>
                      </div>
                      <div className="flex justify-between text-sm">
                        <span className="text-muted-foreground">Duration</span>
                        <span className="font-semibold">{course.duration}</span>
                      </div>
                      <div className="flex justify-between text-sm">
                        <span className="text-muted-foreground">Total Lessons</span>
                        <span className="font-semibold">
                          {course.modules.reduce((acc, m) => acc + m.lessons.length, 0)}
                        </span>
                      </div>
                    </div>

                    <Button className="w-full" size="lg">
                      Start Learning
                      <Play className="w-4 h-4 ml-2" />
                    </Button>

                    <Button variant="outline" className="w-full">
                      <Link href="/catalog">Browse Other Courses</Link>
                    </Button>
                  </CardContent>
                </Card>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Course Content */}
      <section className="py-12 px-4">
        <div className="container mx-auto">
          <div className="grid lg:grid-cols-4 gap-8">
            {/* Left: Video & Content */}
            <div className="lg:col-span-3">
              {selectedLesson ? (
                <Tabs defaultValue="video" className="w-full">
                  <TabsList className="grid w-full max-w-md grid-cols-3 mb-6">
                    <TabsTrigger value="video">
                      <Video className="w-4 h-4 mr-2" />
                      Video
                    </TabsTrigger>
                    <TabsTrigger value="content">
                      <BookOpen className="w-4 h-4 mr-2" />
                      Content
                    </TabsTrigger>
                    {selectedLesson.codeTemplate && (
                      <TabsTrigger value="code">
                        <Terminal className="w-4 h-4 mr-2" />
                        Code
                      </TabsTrigger>
                    )}
                  </TabsList>

                  <TabsContent value="video">
                    <Card className="glass">
                      <CardContent className="p-0">
                        {selectedLesson.videoUrl ? (
                          <div className="aspect-video bg-black rounded-lg overflow-hidden">
                            <video
                              src={selectedLesson.videoUrl}
                              controls
                              className="w-full h-full"
                            />
                          </div>
                        ) : (
                          <div className="aspect-video bg-muted rounded-lg flex items-center justify-center">
                            <div className="text-center">
                              <Video className="w-16 h-16 text-muted-foreground mx-auto mb-4" />
                              <p className="text-muted-foreground">Video coming soon</p>
                              <Badge className="mt-2">Placeholder</Badge>
                            </div>
                          </div>
                        )}
                      </CardContent>
                    </Card>

                    <Card className="glass mt-6">
                      <CardHeader>
                        <CardTitle className="flex items-center gap-2">
                          <Play className="w-5 h-5 text-primary" />
                          {selectedLesson.title}
                        </CardTitle>
                        <CardDescription>
                          {selectedLesson.videoDuration && (
                            <span className="flex items-center gap-1">
                              <Clock className="w-3 h-3" />
                              {formatDuration(selectedLesson.videoDuration)}
                            </span>
                          )}
                        </CardDescription>
                      </CardHeader>
                    </Card>
                  </TabsContent>

                  <TabsContent value="content">
                    <Card className="glass">
                      <CardHeader>
                        <CardTitle className="flex items-center gap-2">
                          <BookOpen className="w-5 h-5 text-primary" />
                          {selectedLesson.title}
                        </CardTitle>
                      </CardHeader>
                      <CardContent>
                        <div className="prose prose-invert max-w-none">
                          <p className="text-lg leading-relaxed">
                            {selectedLesson.content}
                          </p>
                        </div>
                      </CardContent>
                    </Card>
                  </TabsContent>

                  {selectedLesson.codeTemplate && (
                    <TabsContent value="code">
                      <Card className="glass">
                        <CardHeader>
                          <CardTitle className="flex items-center gap-2">
                            <Terminal className="w-5 h-5 text-primary" />
                            Code Exercise
                          </CardTitle>
                          <CardDescription>
                            Practice what you've learned with this coding exercise
                          </CardDescription>
                        </CardHeader>
                        <CardContent>
                          <div className="bg-muted rounded-lg p-4 font-mono text-sm overflow-x-auto">
                            <pre>
                              <code>{selectedLesson.codeTemplate}</code>
                            </pre>
                          </div>
                          <div className="flex gap-2 mt-4">
                            <Button>
                              Run Code
                              <Play className="w-4 h-4 ml-2" />
                            </Button>
                            <Button variant="outline">
                              Open in Code Editor
                              <ChevronRight className="w-4 h-4 ml-2" />
                            </Button>
                          </div>
                        </CardContent>
                      </Card>
                    </TabsContent>
                  )}
                </Tabs>
              ) : (
                <Card className="glass">
                  <CardContent className="py-12 text-center">
                    <BookOpen className="w-16 h-16 text-muted-foreground mx-auto mb-4" />
                    <h3 className="text-xl font-semibold mb-2">Select a lesson</h3>
                    <p className="text-muted-foreground">
                      Choose a lesson from the sidebar to start learning
                    </p>
                  </CardContent>
                </Card>
              )}
            </div>

            {/* Right: Lesson List */}
            <div className="lg:col-span-1">
              <Card className="glass sticky top-24">
                <CardHeader>
                  <CardTitle className="text-lg">Course Content</CardTitle>
                  <CardDescription>
                    {course.modules.length} modules • {course.modules.reduce((acc, m) => acc + m.lessons.length, 0)} lessons
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <ScrollArea className="h-[calc(100vh-250px)]">
                    <div className="space-y-4">
                      {course.modules.map((module, moduleIndex) => (
                        <div key={module.id}>
                          <div className="flex items-center gap-2 mb-2">
                            <div className="w-6 h-6 rounded-full bg-primary/20 flex items-center justify-center text-xs font-semibold text-primary">
                              {moduleIndex + 1}
                            </div>
                            <h4 className="font-semibold text-sm">{module.title}</h4>
                          </div>

                          <div className="space-y-1 ml-8">
                            {module.lessons.map((lesson, lessonIndex) => {
                              const isSelected = selectedLesson?.id === lesson.id
                              return (
                                <motion.button
                                  key={lesson.id}
                                  onClick={() => selectLesson(moduleIndex, lessonIndex)}
                                  className={`w-full text-left p-3 rounded-lg transition-all duration-200 ${
                                    isSelected
                                      ? 'bg-primary/10 border border-primary/50'
                                      : 'hover:bg-muted/50 border border-transparent'
                                  }`}
                                  whileHover={{ x: 4 }}
                                >
                                  <div className="flex items-center gap-2">
                                    {isSelected ? (
                                      <CheckCircle2 className="w-4 h-4 text-primary flex-shrink-0" />
                                    ) : (
                                      <Play className="w-4 h-4 text-muted-foreground flex-shrink-0" />
                                    )}
                                    <span className="text-sm flex-1 line-clamp-1">
                                      {lesson.title}
                                    </span>
                                    {lesson.videoDuration && (
                                      <span className="text-xs text-muted-foreground flex-shrink-0">
                                        {formatDuration(lesson.videoDuration)}
                                      </span>
                                    )}
                                  </div>
                                </motion.button>
                              )
                            })}
                          </div>
                        </div>
                      ))}
                    </div>
                  </ScrollArea>
                </CardContent>
              </Card>
            </div>
          </div>
        </div>
      </section>
    </div>
  )
}
