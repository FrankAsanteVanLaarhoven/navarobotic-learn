import type { Metadata } from "next";
import { Inter } from "next/font/google";
import { JetBrains_Mono } from "next/font/google";
import "./globals.css";
import { Toaster } from "@/components/ui/toaster";
import { ThemeProvider } from "next-themes";
import { Suspense } from "react";
import Loading from "./loading";
import { SoundProvider } from "@/components/SoundProvider";
import { VoiceNavigator } from "@/components/VoiceNavigator";

const inter = Inter({
  variable: "--font-inter",
  subsets: ["latin"],
  display: "swap",
});

const jetbrainsMono = JetBrains_Mono({
  variable: "--font-jetbrains-mono",
  subsets: ["latin"],
  display: "swap",
});

export const metadata: Metadata = {
  title: "Rovyn — Learn robotics by building",
  description: "Courses, 3D simulation, and real-robot labs in one platform.",
  keywords: ["Robotics", "Learn robotics", "ROS", "Simulation", "Robot programming", "3D simulation", "Robotics courses"],
  authors: [{ name: "Rovyn" }],
  icons: {
    icon: [
      { url: "/favicon-r.png", sizes: "32x32", type: "image/png" },
      { url: "/logos/logo-r-icon.svg", sizes: "32x32", type: "image/svg+xml" },
    ],
  },
  openGraph: {
    title: "Rovyn",
    description: "Learn robotics by building.",
    url: "https://rovyn.io",
    siteName: "Rovyn",
    type: "website",
  },
  twitter: {
    card: "summary_large_image",
    title: "Rovyn",
    description: "Learn robotics by building.",
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
        className={`${inter.variable} ${jetbrainsMono.variable} antialiased bg-background text-foreground font-sans`}
      >
        <ThemeProvider attribute="class" defaultTheme="system" enableSystem>
          <SoundProvider>
            <Suspense fallback={<Loading />}>
              {children}
            </Suspense>
            <VoiceNavigator />
            <Toaster />
          </SoundProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}
