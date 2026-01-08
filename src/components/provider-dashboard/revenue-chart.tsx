'use client'

import { useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { TrendingUp, Calendar } from 'lucide-react'
import { motion } from 'framer-motion'

interface RevenueData {
  month: string
  earnings: number
}

interface RevenueChartProps {
  title: string
  period: string
  data: RevenueData[]
}

export function RevenueChart({ title, period, data }: RevenueChartProps) {
  const [selectedPeriod, setSelectedPeriod] = useState('month')
  const maxEarnings = Math.max(...data.map(d => d.earnings))
  const avgEarnings = data.reduce((sum, d) => sum + d.earnings, 0) / data.length
  const growth = data.length > 1 
    ? ((data[data.length - 1].earnings - data[0].earnings) / data[0].earnings) * 100
    : 0

  return (
    <Card className="glass border-border/50">
      <CardHeader className="flex items-center justify-between">
        <div className="flex items-center gap-3">
          <div className="p-2 bg-primary/20 rounded-lg">
            <TrendingUp className="w-6 h-6 text-primary" />
          </div>
          <div>
            <CardTitle>{title}</CardTitle>
            <CardDescription>{period} earnings overview</CardDescription>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <Button
            variant={selectedPeriod === 'month' ? 'default' : 'outline'}
            size="sm"
            onClick={() => setSelectedPeriod('month')}
          >
            Month
          </Button>
          <Button
            variant={selectedPeriod === 'quarter' ? 'default' : 'outline'}
            size="sm"
            onClick={() => setSelectedPeriod('quarter')}
          >
            Quarter
          </Button>
          <Button
            variant={selectedPeriod === 'year' ? 'default' : 'outline'}
            size="sm"
            onClick={() => setSelectedPeriod('year')}
          >
            Year
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        {/* Main Stats */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
          <div className="p-4 bg-muted/50 rounded-lg border border-border/50">
            <div className="text-xs text-muted-foreground mb-1">Total Earnings ({selectedPeriod})</div>
            <div className="text-3xl font-bold">
              ${(data.reduce((sum, d) => sum + d.earnings, 0) / 1000).toFixed(1)}K
            </div>
          </div>

          <div className="p-4 bg-muted/50 rounded-lg border border-border/50">
            <div className="text-xs text-muted-foreground mb-1">Average ({selectedPeriod})</div>
            <div className="text-3xl font-bold">
              ${(avgEarnings / 1000).toFixed(1)}K
            </div>
          </div>

          <div className="p-4 bg-muted/50 rounded-lg border border-border/50">
            <div className="text-xs text-muted-foreground mb-1">Peak ({selectedPeriod})</div>
            <div className="text-3xl font-bold">
              {data.find((d) => d.earnings === maxEarnings)?.month || 'N/A'}
            </div>
            <div className="text-xs text-muted-foreground mt-1">
              ${(maxEarnings / 1000).toFixed(1)}K
            </div>
          </div>
        </div>

        {/* Revenue Chart */}
        <div className="mt-8 p-6 bg-muted/30 rounded-lg border border-border/50">
          <div className="flex items-center justify-between mb-4">
            <div className="text-lg font-semibold">
              Revenue Trend
            </div>
            <div className="text-xs text-muted-foreground">
              Showing {selectedPeriod} data
            </div>
          </div>

          <div className="h-64 flex items-end gap-2">
            {data.map((d, idx) => {
              const barHeight = (d.earnings / maxEarnings) * 100
              
              return (
                <div key={idx} className="flex-1 flex flex-col items-center">
                  {/* Month Label */}
                  <div className="text-xs text-muted-foreground text-center mb-2">
                    {d.month}
                  </div>

                  {/* Bar */}
                  <div className="relative w-full h-48 bg-muted/50 rounded-t-lg overflow-hidden group">
                    <motion.div
                      className="absolute bottom-0 left-0 right-0 h-full bg-gradient-to-t from-primary to-secondary rounded-b-lg"
                      initial={{ height: 0 }}
                      animate={{ height: `${barHeight}%` }}
                      transition={{ duration: 0.5, delay: idx * 0.1 }}
                    />
                    
                    {/* Earnings Value */}
                    <div className="absolute top-2 left-2 right-2 z-10 opacity-0 group-hover:opacity-100 transition-opacity">
                      <div className="bg-card/90 backdrop-blur-sm rounded-lg p-2 border border-border/50">
                        <div className="text-sm font-semibold">
                          ${(d.earnings / 1000).toFixed(1)}K
                        </div>
                        <div className={`text-xs ${growth >= 0 ? 'text-primary' : 'text-destructive'}`}>
                          {growth > 0 ? '+' : ''}{growth.toFixed(1)}%
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              )
            })}
          </div>

          {/* Growth Rate */}
          <div className="mt-6 flex items-center justify-center gap-4">
            <div className="text-center">
              <div className="text-3xl font-bold text-primary mb-1">
                {growth > 0 ? '+' : ''}{growth.toFixed(1)}%
              </div>
              <div className="flex items-center gap-1 text-sm text-muted-foreground">
                <Calendar className="w-4 h-4" />
                <span>Growth Rate / {selectedPeriod}</span>
              </div>
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
