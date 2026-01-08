'use client'

import { useState, useEffect } from 'react'
import { useTheme } from 'next-themes'
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu'
import { Button } from '@/components/ui/button'
import { Label } from '@/components/ui/label'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Sun, Moon, Monitor, Palette, ChevronDown } from 'lucide-react'

const PRESET_COLORS = [
  { name: 'Reddish Orange', value: '#ff6b35', default: true },
  { name: 'Bright Cyan', value: '#06b6d4' },
  { name: 'Electric Blue', value: '#2E7DFF' },
  { name: 'Lime Green', value: '#84cc16' },
  { name: 'Amber', value: '#f59e0b' },
  { name: 'Pink', value: '#ec4899' },
  { name: 'Purple', value: '#a855f7' },
  { name: 'White', value: '#ffffff' },
  { name: 'Yellow', value: '#eab308' },
  { name: 'Teal', value: '#14b8a6' },
]

export function ThemeCustomizer() {
  const { theme, setTheme } = useTheme()
  const [mounted, setMounted] = useState(false)
  const [customColor, setCustomColor] = useState('#ff6b35')
  const [haloColor, setHaloColor] = useState('rgba(255, 107, 53, 0.8)')
  const [backgroundColor, setBackgroundColor] = useState('transparent')
  const [gradientStart, setGradientStart] = useState('#ff6b35')
  const [gradientEnd, setGradientEnd] = useState('#ff6b35')
  const [useGradient, setUseGradient] = useState(false)
  const [isOpen, setIsOpen] = useState(false)

  useEffect(() => {
    setMounted(true)
    // Load saved colors from localStorage
    const savedTextColor = localStorage.getItem('mission-text-color')
    const savedHaloColor = localStorage.getItem('mission-halo-color')
    const savedBgColor = localStorage.getItem('mission-background-color')
    const savedGradientStart = localStorage.getItem('mission-gradient-start')
    const savedGradientEnd = localStorage.getItem('mission-gradient-end')
    const savedUseGradient = localStorage.getItem('mission-use-gradient')

    if (savedTextColor) {
      setCustomColor(savedTextColor)
      applyTextColor(savedTextColor)
    }
    if (savedHaloColor) {
      setHaloColor(savedHaloColor)
      applyHaloColor(savedHaloColor)
    }
    if (savedBgColor) {
      setBackgroundColor(savedBgColor)
      applyBackgroundColor(savedBgColor)
    }
    if (savedGradientStart) setGradientStart(savedGradientStart)
    if (savedGradientEnd) setGradientEnd(savedGradientEnd)
    if (savedUseGradient === 'true') {
      setUseGradient(true)
      applyGradient(savedGradientStart || '#ff6b35', savedGradientEnd || '#ff6b35')
    }
  }, [])

  const applyTextColor = (color: string) => {
    document.documentElement.style.setProperty('--mission-text-color', color)
    localStorage.setItem('mission-text-color', color)
  }

  const applyHaloColor = (color: string) => {
    // Convert hex to rgba if needed
    let rgbaColor = color
    if (color.startsWith('#')) {
      const r = parseInt(color.slice(1, 3), 16)
      const g = parseInt(color.slice(3, 5), 16)
      const b = parseInt(color.slice(5, 7), 16)
      rgbaColor = `rgba(${r}, ${g}, ${b}, 0.8)`
    }
    document.documentElement.style.setProperty('--mission-halo-color', rgbaColor)
    localStorage.setItem('mission-halo-color', rgbaColor)
  }

  const applyBackgroundColor = (color: string) => {
    document.documentElement.style.setProperty('--mission-background-color', color)
    localStorage.setItem('mission-background-color', color)
  }

  const applyGradient = (start: string, end: string) => {
    const gradient = `linear-gradient(135deg, ${start}, ${end})`
    document.documentElement.style.setProperty('--mission-background-gradient', gradient)
    document.documentElement.style.setProperty('--mission-gradient-start', start)
    document.documentElement.style.setProperty('--mission-gradient-end', end)
    localStorage.setItem('mission-gradient-start', start)
    localStorage.setItem('mission-gradient-end', end)
    localStorage.setItem('mission-use-gradient', 'true')
    // Force re-render of Mission Impossible text components
    window.dispatchEvent(new Event('mission-gradient-updated'))
  }

  const removeGradient = () => {
    document.documentElement.style.setProperty('--mission-background-gradient', 'transparent')
    localStorage.setItem('mission-use-gradient', 'false')
    window.dispatchEvent(new Event('mission-gradient-updated'))
  }

  const handleTextColorChange = (color: string) => {
    setCustomColor(color)
    applyTextColor(color)
  }

  const handleHaloColorChange = (color: string) => {
    setHaloColor(color)
    applyHaloColor(color)
  }

  const handleBackgroundColorChange = (color: string) => {
    setBackgroundColor(color)
    applyBackgroundColor(color)
  }

  const handleGradientToggle = (enabled: boolean) => {
    setUseGradient(enabled)
    if (enabled) {
      applyGradient(gradientStart, gradientEnd)
    } else {
      removeGradient()
    }
  }

  const handleGradientStartChange = (color: string) => {
    setGradientStart(color)
    if (useGradient) {
      applyGradient(color, gradientEnd)
    }
  }

  const handleGradientEndChange = (color: string) => {
    setGradientEnd(color)
    if (useGradient) {
      applyGradient(gradientStart, color)
    }
  }

  if (!mounted) return null

  return (
    <DropdownMenu open={isOpen} onOpenChange={setIsOpen}>
      <DropdownMenuTrigger asChild>
        <Button
          variant="ghost"
          size="sm"
          className="relative glass rounded-xl border-border/50 hover:border-primary/50 transition-all"
        >
          <Palette className="w-4 h-4 mr-2" />
          <span className="hidden sm:inline">Theme</span>
          <ChevronDown className="w-3 h-3 ml-1" />
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent
        align="end"
        className="w-[90vw] sm:w-[500px] max-h-[90vh] overflow-y-auto p-0 glass border-border/50 rounded-xl"
      >
        <div className="p-4 border-b border-border/50">
          <h3 className="font-semibold text-lg mb-4">Theme & Color Customization</h3>
          
          <Tabs defaultValue="theme" className="w-full">
            <TabsList className="w-full grid grid-cols-4">
              <TabsTrigger 
                value="theme" 
                className="text-xs transition-all duration-200 hover:bg-accent/50 hover:scale-105 data-[state=active]:bg-primary data-[state=active]:text-primary-foreground"
              >
                Theme
              </TabsTrigger>
              <TabsTrigger 
                value="text-color" 
                className="text-xs transition-all duration-200 hover:bg-accent/50 hover:scale-105 data-[state=active]:bg-primary data-[state=active]:text-primary-foreground"
              >
                Text
              </TabsTrigger>
              <TabsTrigger 
                value="halo" 
                className="text-xs transition-all duration-200 hover:bg-accent/50 hover:scale-105 data-[state=active]:bg-primary data-[state=active]:text-primary-foreground"
              >
                Halo
              </TabsTrigger>
              <TabsTrigger 
                value="gradient" 
                className="text-xs transition-all duration-200 hover:bg-accent/50 hover:scale-105 data-[state=active]:bg-primary data-[state=active]:text-primary-foreground"
              >
                Gradient
              </TabsTrigger>
            </TabsList>

            <TabsContent value="theme" className="mt-4 space-y-3">
              <div>
                <Label className="text-sm font-medium mb-2 block">Appearance</Label>
                <div className="grid grid-cols-3 gap-2">
                  <Button
                    variant={theme === 'light' ? 'default' : 'outline'}
                    className="flex flex-col items-center gap-2 h-auto py-3 transition-all duration-200 hover:scale-105 hover:shadow-md hover:border-primary/50 group"
                    onClick={() => setTheme('light')}
                  >
                    <Sun className="w-5 h-5 transition-transform duration-200 group-hover:rotate-180" />
                    <span className="text-xs">Light</span>
                  </Button>
                  <Button
                    variant={theme === 'dark' ? 'default' : 'outline'}
                    className="flex flex-col items-center gap-2 h-auto py-3 transition-all duration-200 hover:scale-105 hover:shadow-md hover:border-primary/50 group"
                    onClick={() => setTheme('dark')}
                  >
                    <Moon className="w-5 h-5 transition-transform duration-200 group-hover:scale-110" />
                    <span className="text-xs">Dark</span>
                  </Button>
                  <Button
                    variant={theme === 'system' ? 'default' : 'outline'}
                    className="flex flex-col items-center gap-2 h-auto py-3 transition-all duration-200 hover:scale-105 hover:shadow-md hover:border-primary/50 group"
                    onClick={() => setTheme('system')}
                  >
                    <Monitor className="w-5 h-5 transition-transform duration-200 group-hover:scale-110" />
                    <span className="text-xs">System</span>
                  </Button>
                </div>
              </div>
              <div className="text-xs text-muted-foreground pt-2 border-t border-border/50">
                Current: <span className="font-semibold capitalize">{theme || 'system'}</span>
              </div>
            </TabsContent>

            <TabsContent value="text-color" className="mt-4 space-y-4">
              <div>
                <Label className="text-sm font-medium mb-2 block">Text Color</Label>
                <div className="flex items-center gap-3">
                  <input
                    type="color"
                    value={customColor}
                    onChange={(e) => handleTextColorChange(e.target.value)}
                    className="w-16 h-16 rounded-lg border-2 border-border cursor-pointer transition-all duration-200 hover:scale-110 hover:shadow-lg hover:border-primary/50"
                    title="Pick a color"
                  />
                  <div className="flex-1">
                    <input
                      type="text"
                      value={customColor}
                      onChange={(e) => handleTextColorChange(e.target.value)}
                      className="w-full px-3 py-2 rounded-lg border border-border bg-background text-foreground font-mono text-sm transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                      placeholder="#ff6b35"
                    />
                  </div>
                </div>
              </div>

              <div>
                <Label className="text-sm font-medium mb-2 block">Background Color</Label>
                <div className="flex items-center gap-3">
                  <input
                    type="color"
                    value={backgroundColor === 'transparent' ? '#000000' : backgroundColor}
                    onChange={(e) => handleBackgroundColorChange(e.target.value)}
                    className="w-16 h-16 rounded-lg border-2 border-border cursor-pointer transition-all duration-200 hover:scale-110 hover:shadow-lg hover:border-primary/50"
                    title="Pick background color"
                  />
                  <div className="flex-1">
                    <input
                      type="text"
                      value={backgroundColor}
                      onChange={(e) => handleBackgroundColorChange(e.target.value)}
                      className="w-full px-3 py-2 rounded-lg border border-border bg-background text-foreground font-mono text-sm transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                      placeholder="transparent or #hex"
                    />
                  </div>
                </div>
              </div>

              <div>
                <Label className="text-sm font-medium mb-2 block">Preset Text Colors</Label>
                <div className="grid grid-cols-5 gap-2">
                  {PRESET_COLORS.map((preset) => (
                    <button
                      key={preset.value}
                      onClick={() => handleTextColorChange(preset.value)}
                      className={`relative aspect-square rounded-lg border-2 transition-all duration-200 hover:scale-110 hover:shadow-lg hover:z-10 ${
                        customColor === preset.value
                          ? 'border-primary scale-110 shadow-lg ring-2 ring-primary/50'
                          : 'border-border hover:border-primary/50'
                      }`}
                      style={{ backgroundColor: preset.value }}
                      title={preset.name}
                    >
                      {preset.default && (
                        <span className="absolute -top-1 -right-1 w-3 h-3 bg-primary rounded-full border-2 border-background" />
                      )}
                    </button>
                  ))}
                </div>
              </div>
            </TabsContent>

            <TabsContent value="halo" className="mt-4 space-y-4">
              <div>
                <Label className="text-sm font-medium mb-2 block">Halo/Glow Color</Label>
                <p className="text-xs text-muted-foreground mb-3">
                  Controls the glow effect around Mission Impossible text
                </p>
                <div className="flex items-center gap-3">
                  <input
                    type="color"
                    value={haloColor.startsWith('rgba') ? '#ff6b35' : haloColor}
                    onChange={(e) => handleHaloColorChange(e.target.value)}
                    className="w-16 h-16 rounded-lg border-2 border-border cursor-pointer transition-all duration-200 hover:scale-110 hover:shadow-lg hover:border-primary/50"
                    title="Pick halo color"
                  />
                  <div className="flex-1">
                    <input
                      type="text"
                      value={haloColor}
                      onChange={(e) => handleHaloColorChange(e.target.value)}
                      className="w-full px-3 py-2 rounded-lg border border-border bg-background text-foreground font-mono text-sm transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                      placeholder="rgba(255, 107, 53, 0.8) or #hex"
                    />
                  </div>
                </div>
              </div>

              <div>
                <Label className="text-sm font-medium mb-2 block">Preset Halo Colors</Label>
                <div className="grid grid-cols-5 gap-2">
                  {PRESET_COLORS.map((preset) => (
                    <button
                      key={`halo-${preset.value}`}
                      onClick={() => {
                        const r = parseInt(preset.value.slice(1, 3), 16)
                        const g = parseInt(preset.value.slice(3, 5), 16)
                        const b = parseInt(preset.value.slice(5, 7), 16)
                        handleHaloColorChange(`rgba(${r}, ${g}, ${b}, 0.8)`)
                      }}
                      className={`relative aspect-square rounded-lg border-2 transition-all duration-200 hover:scale-110 hover:shadow-lg hover:z-10 ${
                        haloColor.includes(preset.value.slice(1))
                          ? 'border-primary scale-110 shadow-lg ring-2 ring-primary/50'
                          : 'border-border hover:border-primary/50'
                      }`}
                      style={{ backgroundColor: preset.value }}
                      title={preset.name}
                    />
                  ))}
                </div>
              </div>
            </TabsContent>

            <TabsContent value="gradient" className="mt-4 space-y-4">
              <div className="flex items-center justify-between">
                <div>
                  <Label className="text-sm font-medium mb-1 block">Enable Gradient</Label>
                  <p className="text-xs text-muted-foreground">
                    Apply gradient colors to text background
                  </p>
                </div>
                <Button
                  variant={useGradient ? 'default' : 'outline'}
                  size="sm"
                  onClick={() => handleGradientToggle(!useGradient)}
                  className="transition-all duration-200 hover:scale-105 hover:shadow-md"
                >
                  {useGradient ? 'Enabled' : 'Disabled'}
                </Button>
              </div>

              {useGradient && (
                <>
                  <div>
                    <Label className="text-sm font-medium mb-2 block">Gradient Start Color</Label>
                    <div className="flex items-center gap-3">
                      <input
                        type="color"
                        value={gradientStart}
                        onChange={(e) => handleGradientStartChange(e.target.value)}
                        className="w-16 h-16 rounded-lg border-2 border-border cursor-pointer transition-all duration-200 hover:scale-110 hover:shadow-lg hover:border-primary/50"
                        title="Gradient start"
                      />
                      <div className="flex-1">
                        <input
                          type="text"
                          value={gradientStart}
                          onChange={(e) => handleGradientStartChange(e.target.value)}
                          className="w-full px-3 py-2 rounded-lg border border-border bg-background text-foreground font-mono text-sm transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                          placeholder="#ff6b35"
                        />
                      </div>
                    </div>
                  </div>

                  <div>
                    <Label className="text-sm font-medium mb-2 block">Gradient End Color</Label>
                    <div className="flex items-center gap-3">
                      <input
                        type="color"
                        value={gradientEnd}
                        onChange={(e) => handleGradientEndChange(e.target.value)}
                        className="w-16 h-16 rounded-lg border-2 border-border cursor-pointer transition-all duration-200 hover:scale-110 hover:shadow-lg hover:border-primary/50"
                        title="Gradient end"
                      />
                      <div className="flex-1">
                        <input
                          type="text"
                          value={gradientEnd}
                          onChange={(e) => handleGradientEndChange(e.target.value)}
                          className="w-full px-3 py-2 rounded-lg border border-border bg-background text-foreground font-mono text-sm transition-all duration-200 hover:border-primary/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
                          placeholder="#06b6d4"
                        />
                      </div>
                    </div>
                  </div>

                  <div className="p-4 rounded-lg border border-border bg-muted/30">
                    <div className="text-xs text-muted-foreground mb-2">Preview</div>
                    <div
                      className="h-12 rounded-lg"
                      style={{
                        background: `linear-gradient(135deg, ${gradientStart}, ${gradientEnd})`
                      }}
                    />
                  </div>

                  <div>
                    <Label className="text-sm font-medium mb-2 block">Quick Gradient Presets</Label>
                    <div className="grid grid-cols-2 gap-2">
                      {[
                        { name: 'Orange to Cyan', start: '#ff6b35', end: '#06b6d4' },
                        { name: 'Blue to Purple', start: '#2E7DFF', end: '#a855f7' },
                        { name: 'Green to Teal', start: '#84cc16', end: '#14b8a6' },
                        { name: 'Pink to Purple', start: '#ec4899', end: '#a855f7' },
                      ].map((preset) => (
                        <button
                          key={preset.name}
                          onClick={() => {
                            setGradientStart(preset.start)
                            setGradientEnd(preset.end)
                            applyGradient(preset.start, preset.end)
                          }}
                          className="p-3 rounded-lg border border-border hover:border-primary/50 hover:bg-accent/30 hover:scale-105 hover:shadow-md transition-all duration-200 text-left group"
                        >
                          <div className="text-xs font-medium mb-2">{preset.name}</div>
                          <div
                            className="h-6 rounded"
                            style={{
                              background: `linear-gradient(135deg, ${preset.start}, ${preset.end})`
                            }}
                          />
                        </button>
                      ))}
                    </div>
                  </div>
                </>
              )}
            </TabsContent>
          </Tabs>
        </div>
      </DropdownMenuContent>
    </DropdownMenu>
  )
}
