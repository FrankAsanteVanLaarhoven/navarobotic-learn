'use client'

import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Progress } from '@/components/ui/progress'
import { Badge } from '@/components/ui/badge'
import { motion } from 'framer-motion'
import { Users, Trophy, TrendingUp, Star, Activity, Zap, Target, Award, Clock } from 'lucide-react'

interface StudentEngagementProps {
  activeStudents: number
  completionRate: number
  averageRating: number
  totalEnrollments: number
  activeCourses: number
  topCourses: {
    id: number
    title: string
    rating: number
    students: number
  }[]
}

export function StudentEngagement({
  activeStudents,
  completionRate,
  averageRating,
  totalEnrollments,
  activeCourses,
  topCourses
}: StudentEngagementProps) {
  return (
    <div className="space-y-6">
      {/* Active Students Card */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5 }}
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
                <div className="text-3xl font-bold">{activeStudents.toLocaleString()}</div>
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
                  <span className="text-3xl font-bold">{completionRate}%</span>
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
                  <span className="text-3xl font-bold">{averageRating}</span>
                </div>
              </div>
              <div className="text-xs text-accent">
                Top 10% of all providers
              </div>
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div>
                <div className="text-sm text-muted-foreground">Total Enrollments</div>
                <div className="text-3xl font-bold">{totalEnrollments.toLocaleString()}</div>
                <div className="text-xs text-muted-foreground mt-1">All time</div>
              </div>
              <div>
                <div className="text-sm text-muted-foreground">Active Courses</div>
                <div className="text-3xl font-bold">{activeCourses}</div>
                <div className="text-xs text-muted-foreground mt-1">Currently published</div>
              </div>
            </div>
          </CardContent>

          <div className="p-4">
            <Progress value={completionRate} className="h-2" />
          </div>
        </Card>
      </motion.div>

      {/* Top Performing Courses */}
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, delay: 0.1 }}
      >
        <Card className="glass border-border/50">
          <CardHeader>
            <div className="flex items-center gap-3">
              <div className="p-2 bg-accent/20 rounded-lg">
                <Trophy className="w-6 h-6 text-accent" />
              </div>
              <CardTitle>Top Performing Courses</CardTitle>
            </div>
          </CardHeader>
          <CardContent>
            <CardDescription className="mb-6">
              Your most popular courses by student engagement and performance
            </CardDescription>

            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              {topCourses.map((course, idx) => (
                <motion.div
                  key={course.id}
                  initial={{ opacity: 0, scale: 0.9 }}
                  animate={{ opacity: 1, scale: 1 }}
                  transition={{ duration: 0.5, delay: idx * 0.1 }}
                  className="space-y-3 p-4 rounded-lg bg-muted/50 hover:bg-muted transition-colors border border-border/50"
                >
                  <div className="flex items-start justify-between mb-2">
                    <div className="flex items-center gap-3">
                      <Trophy className="w-5 h-5 text-accent" />
                      <div>
                        <div className="font-semibold mb-1">{course.title}</div>
                        <div className="text-xs text-muted-foreground">{course.students.toLocaleString()} students</div>
                      </div>
                    </div>
                    <Badge className="bg-accent/20 text-accent border-accent/50">
                      #{idx + 1}
                    </Badge>
                  </div>

                  <div className="space-y-2">
                    <div className="flex items-center gap-2">
                      <Star className="w-4 h-4 text-accent fill-accent" />
                      <span className="text-2xl font-bold">{course.rating}</span>
                      <span className="text-xs text-muted-foreground">Rating</span>
                    </div>
                    <div className="flex items-center gap-2">
                      <Activity className="w-4 h-4 text-primary" />
                      <span className="text-lg font-semibold">{course.students.toLocaleString()} Active</span>
                    </div>
                  </div>
                </motion.div>
              ))}
            </div>
          </CardContent>
        </Card>
      </motion.div>

      {/* Engagement Insights */}
      <Card className="glass border-border/50">
        <CardHeader>
          <div className="flex items-center gap-3">
            <div className="p-2 bg-primary/20 rounded-lg">
              <Zap className="w-6 h-6 text-primary" />
            </div>
            <CardTitle>Engagement Insights</CardTitle>
          </div>
        </CardHeader>
        <CardContent className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div className="p-4 rounded-lg bg-muted/50 border border-border/50">
            <Target className="w-8 h-8 text-primary mx-auto mb-3" />
            <div className="text-center">
              <div className="text-3xl font-bold mb-1">78%</div>
              <div className="text-sm text-muted-foreground">Lesson Completion Rate</div>
              <div className="text-xs text-muted-foreground mt-2">
                Students complete 78% of lessons on average
              </div>
            </div>
          </div>

          <div className="p-4 rounded-lg bg-muted/50 border border-border/50">
            <Clock className="w-8 h-8 text-secondary mx-auto mb-3" />
            <div className="text-center">
              <div className="text-3xl font-bold mb-1">4.2 hrs</div>
              <div className="text-sm text-muted-foreground">Average Time Spent Per Week</div>
              <div className="text-xs text-muted-foreground mt-2">
                Students spend 4.2 hours weekly on average
              </div>
            </div>
          </div>

          <div className="p-4 rounded-lg bg-muted/50 border border-border/50">
            <Award className="w-8 h-8 text-primary mx-auto mb-3" />
            <div className="text-center">
              <div className="text-3xl font-bold mb-1">15 days</div>
              <div className="text-sm text-muted-foreground">Average Streak Length</div>
              <div className="text-xs text-muted-foreground mt-2">
                Students maintain activity for 15 days on average
              </div>
            </div>
          </div>

          <div className="p-4 rounded-lg bg-muted/50 border border-border/50">
            <Activity className="w-8 h-8 text-secondary mx-auto mb-3" />
            <div className="text-center">
              <div className="text-3xl font-bold mb-1">89%</div>
              <div className="text-sm text-muted-foreground">Satisfaction Score</div>
              <div className="text-xs text-muted-foreground mt-2">
                Based on student reviews and ratings
              </div>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
