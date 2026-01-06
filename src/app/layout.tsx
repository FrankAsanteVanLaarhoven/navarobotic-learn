import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import "./globals.css";
import { Toaster } from "@/components/ui/toaster";
import { Suspense } from "react";
import Loading from "./loading";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "Robotics Training Platform - AI-Powered Learning",
  description: "Build, program, and simulate advanced humanoid robots with our revolutionary platform. Access real robots, 3D simulations, and AI-powered learning paths.",
  keywords: ["Robotics", "Humanoid Robots", "Simulation", "AI", "Next.js", "TypeScript", "Tailwind CSS", "shadcn/ui"],
  authors: [{ name: "Robotics Platform Team" }],
  icons: {
    icon: "https://platform.com/logo.svg",
  },
  openGraph: {
    title: "Robotics Training Platform",
    description: "AI-powered humanoid robotics education platform with real simulations and video generation",
    url: "https://platform.com",
    siteName: "Robotics Platform",
    type: "website",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" suppressHydrationWarning>
      <head>
        {/* Performance: Preconnect to critical domains */}
        <link rel="preconnect" href="https://fonts.googleapis.com" />
        <link rel="dns-prefetch" href="https://fonts.googleapis.com" />
        <link rel="preconnect" href="https://localhost:3000" />
      </head>
      <body
        className={`${geistSans.variable} ${geistMono.variable} antialiased bg-background text-foreground`}
      >
        <Suspense fallback={<Loading />}>
          {children}
        </Suspense>
        <Toaster />
      </body>
    </html>
  );
}
