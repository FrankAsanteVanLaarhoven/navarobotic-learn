'use client'

import { motion } from 'framer-motion'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Input } from '@/components/ui/input'
import {
  BookOpen,
  Clock,
  Users,
  ChevronRight,
  Star,
  Filter,
  Search
} from 'lucide-react'
import { useState, useEffect } from 'react'
import Link from 'next/link'

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
  learningPath?: {
    title: string
  }
}

export default function CatalogPage() {
  const [learningPaths, setLearningPaths] = useState<LearningPath[]>([])
  const [allCourses, setAllCourses] = useState<Course[]>([])
  const [loading, setLoading] = useState(true)
  const [searchQuery, setSearchQuery] = useState('')
  const [selectedLevel, setSelectedLevel] = useState<string>('all')

  useEffect(() => {
    fetchData()
  }, [])

  const fetchData = async () => {
    try {
      const [pathsRes, coursesRes] = await Promise.all([
        fetch('/api/learning-paths'),
        fetch('/api/courses')
      ])
      const pathsData = await pathsRes.json()
      const coursesData = await coursesRes.json()
      setLearningPaths(pathsData)
      setAllCourses(coursesData)
    } catch (error) {
      console.error('Error fetching data:', error)
    } finally {
      setLoading(false)
    }
  }

  const filteredCourses = allCourses.filter(course => {
    const matchesSearch = course.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
                         course.description.toLowerCase().includes(searchQuery.toLowerCase())
    const matchesLevel = selectedLevel === 'all' || course.level.toLowerCase() === selectedLevel.toLowerCase()
    return matchesSearch && matchesLevel
  })

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

  if (loading) {
    return (
      <div className="min-h-screen flex items-center justify-center">
        <div className="text-center">
          <div className="inline-block animate-pulse-slow">
            <div className="w-16 h-16 rounded-full bg-primary/20 flex items-center justify-center mx-auto mb-4">
              <BookOpen className="w-8 h-8 text-primary animate-pulse" />
            </div>
          </div>
          <p className="text-lg text-muted-foreground">Loading catalog...</p>
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen">
      {/* Header */}
      <section className="pt-24 pb-12 px-4 border-b border-border/50">
        <div className="container mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
          >
            <h1 className="text-4xl sm:text-5xl font-bold mb-4">
              Course <span className="gradient-text">Catalog</span>
            </h1>
            <p className="text-lg text-muted-foreground max-w-2xl mb-8">
              Explore our comprehensive library of robotics courses designed for every skill level
            </p>

            {/* Search and Filters */}
            <div className="flex flex-col sm:flex-row gap-4 mb-6">
              <div className="flex-1 relative">
                <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 w-5 h-5 text-muted-foreground" />
                <Input
                  placeholder="Search courses..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  className="pl-10"
                />
              </div>
              <div className="flex gap-2">
                <Button
                  variant={selectedLevel === 'all' ? 'default' : 'outline'}
                  onClick={() => setSelectedLevel('all')}
                  size="sm"
                >
                  All Levels
                </Button>
                <Button
                  variant={selectedLevel === 'beginner' ? 'default' : 'outline'}
                  onClick={() => setSelectedLevel('beginner')}
                  size="sm"
                >
                  Beginner
                </Button>
                <Button
                  variant={selectedLevel === 'intermediate' ? 'default' : 'outline'}
                  onClick={() => setSelectedLevel('intermediate')}
                  size="sm"
                >
                  Intermediate
                </Button>
                <Button
                  variant={selectedLevel === 'advanced' ? 'default' : 'outline'}
                  onClick={() => setSelectedLevel('advanced')}
                  size="sm"
                >
                  Advanced
                </Button>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Content */}
      <section className="py-12 px-4">
        <div className="container mx-auto">
          <Tabs defaultValue="paths" className="w-full">
            <TabsList className="grid w-full max-w-md grid-cols-2 mb-8">
              <TabsTrigger value="paths">Learning Paths</TabsTrigger>
              <TabsTrigger value="courses">All Courses</TabsTrigger>
            </TabsList>

            <TabsContent value="paths">
              <div className="grid lg:grid-cols-3 gap-8">
                {learningPaths.map((path, index) => (
                  <motion.div
                    key={path.id}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.1 }}
                  >
                    <Link href={`/learning-paths/${path.id}`}>
                      <Card className="glass h-full border-border/50 hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                        <CardHeader className={`bg-gradient-to-r ${getPathColor(path.level)} rounded-t-xl`}>
                          <div className="flex items-center justify-between mb-2">
                            <Badge className={`capitalize ${getLevelColor(path.level)}`}>
                              {path.level}
                            </Badge>
                          </div>
                          <CardTitle className="text-2xl group-hover:text-primary transition-colors">
                            {path.title}
                          </CardTitle>
                          <CardDescription>{path.duration}</CardDescription>
                        </CardHeader>
                        <CardContent className="pt-6">
                          <p className="text-sm text-muted-foreground mb-4">
                            {path.description}
                          </p>
                          <div className="flex items-center justify-between text-sm text-muted-foreground mb-4">
                            <span>{path.courses.length} courses</span>
                          </div>
                          <Button className="w-full" variant="outline">
                            View Path
                            <ChevronRight className="w-4 h-4 ml-2" />
                          </Button>
                        </CardContent>
                      </Card>
                    </Link>
                  </motion.div>
                ))}
              </div>
            </TabsContent>

            <TabsContent value="courses">
              {filteredCourses.length === 0 ? (
                <div className="text-center py-12">
                  <Search className="w-16 h-16 text-muted-foreground mx-auto mb-4" />
                  <h3 className="text-xl font-semibold mb-2">No courses found</h3>
                  <p className="text-muted-foreground">
                    Try adjusting your search or filters
                  </p>
                </div>
              ) : (
                <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
                  {filteredCourses.map((course, index) => (
                    <motion.div
                      key={course.id}
                      initial={{ opacity: 0, y: 20 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ delay: index * 0.05 }}
                    >
                      <Link href={`/courses/${course.slug}`}>
                        <Card className="glass h-full border-border/50 hover:border-primary/50 transition-all duration-300 cursor-pointer group">
                          {course.thumbnail && (
                            <div className="relative h-48 overflow-hidden">
                              <img
                                src={course.thumbnail}
                                alt={course.title}
                                className="w-full h-full object-cover group-hover:scale-110 transition-transform duration-300"
                              />
                              <div className="absolute inset-0 bg-gradient-to-t from-background to-transparent" />
                              <Badge className="absolute top-4 right-4 capitalize" variant="secondary">
                                {course.level}
                              </Badge>
                            </div>
                          )}
                          <CardHeader>
                            <CardTitle className="text-xl group-hover:text-primary transition-colors">
                              {course.title}
                            </CardTitle>
                            <CardDescription className="line-clamp-2">
                              {course.description}
                            </CardDescription>
                          </CardHeader>
                          <CardContent>
                            <div className="flex items-center gap-4 mb-4 text-sm text-muted-foreground">
                              <div className="flex items-center gap-1">
                                <Clock className="w-4 h-4" />
                                <span>{course.duration}</span>
                              </div>
                              <div className="flex items-center gap-1">
                                <Users className="w-4 h-4" />
                                <span>{course.totalStudents} students</span>
                              </div>
                            </div>
                            <div className="flex items-center justify-between">
                              <div className="flex items-center gap-1">
                                <Star className="w-4 h-4 fill-yellow-500 text-yellow-500" />
                                <span className="font-semibold">{course.rating}</span>
                              </div>
                              <Button size="sm" variant="outline">
                                View
                                <ChevronRight className="w-4 h-4 ml-1" />
                              </Button>
                            </div>
                          </CardContent>
                        </Card>
                      </Link>
                    </motion.div>
                  ))}
                </div>
              )}
            </TabsContent>
          </Tabs>
        </div>
      </section>
    </div>
  )
}
