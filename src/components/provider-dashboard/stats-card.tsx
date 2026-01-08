'use client'

import { ReactNode } from 'react'
import { Card, CardContent } from '@/components/ui/card'
import { TrendingUp, TrendingDown, Minus } from 'lucide-react'

interface StatsCardProps {
  title: string
  value: string | number
  change?: string | number
  changeType?: 'positive' | 'negative' | 'neutral'
  icon: ReactNode
  iconColor?: 'primary' | 'secondary' | 'accent'
}

export function StatsCard({ title, value, change, changeType, icon, iconColor = 'primary' }: StatsCardProps) {
  const iconColors = {
    primary: 'bg-primary/20 text-primary',
    secondary: 'bg-secondary/20 text-secondary',
    accent: 'bg-accent/20 text-accent'
  }

  return (
    <Card className="glass border-border/50 hover:border-primary/50 transition-colors group">
      <CardContent className="p-6">
        <div className="flex items-center justify-between mb-4">
          <div className="flex items-center gap-3">
            <div className={`p-3 rounded-xl ${iconColors[iconColor]}`}>
              {icon}
            </div>
            <div>
              <div className="text-xs text-muted-foreground mb-1">{title}</div>
              <div className="text-3xl font-bold group-hover:text-primary transition-colors">
                {typeof value === 'number' ? value.toLocaleString() : value}
              </div>
            </div>
          </div>
          {change && (
            <div className={`flex items-center gap-2 text-sm ${
              changeType === 'positive' ? 'text-primary' :
              changeType === 'negative' ? 'text-destructive' : 'text-muted-foreground'
            }`}>
              {changeType === 'positive' && <TrendingUp className="w-4 h-4" />}
              {changeType === 'negative' && <TrendingDown className="w-4 h-4" />}
              {changeType === 'neutral' && <Minus className="w-4 h-4" />}
              <span>{changeType === 'positive' ? '+' : changeType === 'negative' ? '-' : ''}{change}</span>
            </div>
          )}
        </div>
        <div className="text-xs text-muted-foreground mt-2">
          {changeType === 'positive' ? 'Increased from last month' :
           changeType === 'negative' ? 'Decreased from last month' :
           'Same as last month'}
        </div>
      </CardContent>
    </Card>
  )
}
