"use client"

import React, { useState } from "react"
import { HoveredLink, Menu, MenuItem, ProductItem } from "@/components/ui/navbar-menu"
import { cn } from "@/lib/utils"
import { AnimatedLogo } from "@/components/AnimatedLogo"
import { ThemeCustomizer } from "@/components/ThemeCustomizer"
import { UtilityMenu } from "@/components/utility-menu"
import { Button } from "@/components/ui/button"
import Link from "next/link"

export function Navbar({ className }: { className?: string }) {
  const [active, setActive] = useState<string | null>(null)
  
  return (
    <div className={cn("fixed top-4 inset-x-0 max-w-7xl mx-auto z-50 px-4", className)}>
      <div className="flex items-center justify-between gap-4">
        {/* Logo */}
        <Link href="/" className="flex items-center gap-2 z-50 flex-shrink-0">
          <AnimatedLogo size="md" />
        </Link>

        {/* Center Menu */}
        <div className="hidden lg:flex flex-1 justify-center">
          <Menu setActive={setActive}>
            <MenuItem setActive={setActive} active={active} item="Courses">
              <div className="text-sm grid grid-cols-2 gap-10 p-4">
                <ProductItem
                  title="Unitree G1 Fundamentals"
                  href="/courses/unitree-g1-fundamentals"
                  src="/images/unitree-g1.png"
                  description="Master the G1 humanoid robot: walking control, navigation, and perception"
                />
                <ProductItem
                  title="Python for Robotics"
                  href="/courses/python-robotics"
                  src="/images/ros2-diagram.png"
                  description="Build and program your own humanoid robot from scratch"
                />
                <ProductItem
                  title="ROS2 & AI Integration"
                  href="/courses/ros2-ai-integration"
                  src="/images/simulation-interface.png"
                  description="Advanced ROS2 programming with AI algorithms for humanoid robots"
                />
                <ProductItem
                  title="Control Systems"
                  href="/courses/control-systems"
                  src="/images/learning-dashboard.png"
                  description="Learn advanced control systems and kinematics for robotics"
                />
              </div>
            </MenuItem>
            <MenuItem setActive={setActive} active={active} item="Learning Paths">
              <div className="flex flex-col space-y-4 text-sm p-4">
                <HoveredLink href="/catalog#paths">
                  <div className="flex items-center gap-3">
                    <div className="w-12 h-12 rounded-lg bg-green-500/20 flex items-center justify-center">
                      <span className="text-green-500 font-bold">B</span>
                    </div>
                    <div>
                      <div className="font-semibold">Beginner Path</div>
                      <div className="text-xs text-muted-foreground">3 Months • Robotics Fundamentals</div>
                    </div>
                  </div>
                </HoveredLink>
                <HoveredLink href="/catalog#paths">
                  <div className="flex items-center gap-3">
                    <div className="w-12 h-12 rounded-lg bg-blue-500/20 flex items-center justify-center">
                      <span className="text-blue-500 font-bold">I</span>
                    </div>
                    <div>
                      <div className="font-semibold">Intermediate Path</div>
                      <div className="text-xs text-muted-foreground">6 Months • Humanoid Development</div>
                    </div>
                  </div>
                </HoveredLink>
                <HoveredLink href="/catalog#paths">
                  <div className="flex items-center gap-3">
                    <div className="w-12 h-12 rounded-lg bg-purple-500/20 flex items-center justify-center">
                      <span className="text-purple-500 font-bold">A</span>
                    </div>
                    <div>
                      <div className="font-semibold">Advanced Path</div>
                      <div className="text-xs text-muted-foreground">9 Months • AI & Robotics Mastery</div>
                    </div>
                  </div>
                </HoveredLink>
              </div>
            </MenuItem>
            <MenuItem setActive={setActive} active={active} item="AI Video Studio">
              <div className="text-sm grid grid-cols-2 gap-10 p-4">
                <ProductItem
                  title="Generate Videos"
                  href="/ai-video/generate"
                  src="/images/feature-1.png"
                  description="Create AI-powered educational videos with multiple models"
                />
                <ProductItem
                  title="Video Library"
                  href="/video"
                  src="/images/feature-2.png"
                  description="Browse and manage your generated video content"
                />
                <ProductItem
                  title="Voice Selection"
                  href="/ai-video/generate"
                  src="/images/feature-3.png"
                  description="Choose from 6+ natural TTS voices for your videos"
                />
                <ProductItem
                  title="7 AI Models"
                  href="/ai-video/generate"
                  src="/images/feature-4.png"
                  description="Sora, Kling, Pika, Runway and more video generation models"
                />
              </div>
            </MenuItem>
            <MenuItem setActive={setActive} active={active} item="Spatial Simulation">
              <div className="text-sm grid grid-cols-2 gap-10 p-4">
                <ProductItem
                  title="3D Robot Simulation"
                  href="/simulation"
                  src="/images/simulation-interface.png"
                  description="Real-time 3D simulation with physics and ROS integration"
                />
                <ProductItem
                  title="Digital Twin"
                  href="/simulation"
                  src="/images/feature-5.png"
                  description="Bi-directional sync between simulation and real robots"
                />
                <ProductItem
                  title="Multi-Robot Scene"
                  href="/simulation"
                  src="/images/image_b7f2b3f6-b5d2-4710-8e3d-d052fd4c78d2.png"
                  description="Simulate multiple robots in a shared environment"
                />
                <ProductItem
                  title="Real Robot Control"
                  href="/simulation"
                  src="/images/hero-robot.png"
                  description="Deploy code directly to physical hardware"
                />
              </div>
            </MenuItem>
          </Menu>
        </div>

        {/* Right Side Actions */}
        <div className="flex items-center gap-2 sm:gap-4 flex-shrink-0">
          <ThemeCustomizer />
          <UtilityMenu />
          <Button variant="ghost" size="sm" className="hidden sm:flex">Sign In</Button>
          <Button size="sm" className="gradient-border hidden sm:flex">Get Started</Button>
        </div>
      </div>
    </div>
  )
}
