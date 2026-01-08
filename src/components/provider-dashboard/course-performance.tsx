'use client'

import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Progress } from '@/components/ui/progress'
import { Button } from '@/components/ui/button'
import { Trophy, Star, TrendingUp, Eye, MoreHorizontal, Users, DollarSign } from 'lucide-react'

interface Course {
  id: number
  title: string
  students: number
  rating: number
  completion: number
  revenue: number
  category: string
  modules: number
  status: string
}

interface CoursePerformanceProps {
  courses: Course[]
  showAll?: boolean
}

export function CoursePerformance({ courses, showAll = false }: CoursePerformanceProps) {
  const displayCourses = showAll ? courses : courses.slice(0, 4)

  return (
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
          {showAll ? (
            <Badge className="bg-secondary/20 text-secondary border-secondary/50 text-xs">
              Show All {displayCourses.length}
            </Badge>
          ) : (
            <Badge className="bg-secondary/20 text-secondary border-secondary/50 text-xs">
              Top {displayCourses.length}
            </Badge>
          )}
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
              {displayCourses.map((course) => (
                <tr key={course.id} className="border-b border-border/50 hover:bg-muted/30 transition-colors">
                  <td className="p-4">
                    <div className="flex items-center gap-3 mb-2">
                      <div className="flex-1">
                        <div className="font-semibold mb-1">{course.title}</div>
                        <Badge variant="outline" className="text-xs">
                          {course.category}
                        </Badge>
                      </div>
                    </div>
                  </td>
                  <td className="p-4">
                    <div className="flex items-center gap-2">
                      <Users className="w-4 h-4 text-muted-foreground" />
                      <span className="text-lg font-semibold">{course.students.toLocaleString()}</span>
                    </div>
                  </td>
                  <td className="p-4">
                    <div className="flex items-center gap-2">
                      <Star className="w-5 h-5 text-accent fill-accent" />
                      <span className="text-lg font-semibold">{course.rating}</span>
                    </div>
                  </td>
                  <td className="p-4">
                    <div className="flex items-center gap-2">
                      <Progress value={course.completion} className="w-20 h-2" />
                      <span className="text-lg font-semibold ml-2">{course.completion}%</span>
                    </div>
                  </td>
                  <td className="p-4">
                    <div className="flex items-center gap-2">
                      <div className="p-2 bg-primary/20 rounded-lg">
                        <DollarSign className="w-4 h-4 text-primary" />
                      </div>
                      <span className="text-lg font-semibold">${course.revenue.toLocaleString()}</span>
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

        {!showAll && courses.length > 4 && (
          <div className="flex justify-center mt-6">
            <Button variant="outline">
              View All {courses.length} Courses
            </Button>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
