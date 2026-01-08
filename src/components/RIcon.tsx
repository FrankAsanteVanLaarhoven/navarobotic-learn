'use client'

interface RIconProps {
  className?: string
  size?: number
}

export function RIcon({ className = '', size = 48 }: RIconProps) {
  return (
    <div className={`flex items-center justify-center ${className}`}>
      <div 
        className="rounded-lg flex items-center justify-center"
        style={{ 
          width: size,
          height: size,
          background: 'linear-gradient(135deg, rgba(46, 125, 255, 0.2), rgba(46, 125, 255, 0.1))',
          border: '1px solid rgba(46, 125, 255, 0.3)',
          boxShadow: '0 0 10px rgba(46, 125, 255, 0.3), 0 0 20px rgba(46, 125, 255, 0.1)'
        }}
      >
        <span 
          className="font-bold"
          style={{
            fontSize: `${size * 0.5}px`,
            color: 'var(--mission-text-color, #2E7DFF)',
            textShadow: `
              0 0 8px var(--mission-halo-color, rgba(46, 125, 255, 0.8)),
              0 0 16px var(--mission-halo-color, rgba(46, 125, 255, 0.6)),
              0 0 24px var(--mission-halo-color, rgba(46, 125, 255, 0.4))
            `,
            fontFamily: 'system-ui, -apple-system, sans-serif',
            letterSpacing: '-0.02em',
            fontWeight: 700
          }}
        >
          R
        </span>
      </div>
    </div>
  )
}
