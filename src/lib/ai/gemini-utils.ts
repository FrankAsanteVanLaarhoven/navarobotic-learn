/**
 * Gemini API Utilities
 * Helper functions for using all Gemini capabilities
 */

import { geminiCompleteService } from './gemini-complete-service'

/**
 * Generate course content using Gemini 3 Flash
 */
export async function generateCourseContent(prompt: string) {
  return await geminiCompleteService.generateText(prompt, {
    model: 'gemini-3-flash',
    temperature: 0.7,
    maxTokens: 4000
  })
}

/**
 * Generate course thumbnail using Nano Banana Pro
 */
export async function generateCourseThumbnail(
  courseTitle: string,
  courseDescription: string
) {
  const imagePrompt = `Professional course thumbnail for: ${courseTitle}

Description: ${courseDescription}

Style: Modern, educational, professional, engaging
Colors: Vibrant but professional
Text: Include course title prominently
Layout: Clean, modern design suitable for online learning platform`

  return await geminiCompleteService.generateImage({
    prompt: imagePrompt,
    model: 'nano-banana-pro',
    resolution: '1024x1024',
    style: 'photographic'
  })
}

/**
 * Generate course video with Veo 3.1
 */
export async function generateCourseVideo(
  prompt: string,
  options: {
    duration?: number
    resolution?: '720p' | '1080p' | '4K'
    withSoundEffects?: boolean
  } = {}
) {
  return await geminiCompleteService.generateVideo({
    prompt: prompt,
    model: 'veo-3.1',
    duration: options.duration || 30,
    resolution: options.resolution || '1080p',
    withSoundEffects: options.withSoundEffects || false,
    quality: 'high'
  })
}

/**
 * Generate instructor narration using Gemini TTS
 */
export async function generateInstructorNarration(
  script: string,
  voice: 'default' | 'male' | 'female' = 'default'
) {
  return await geminiCompleteService.generateSpeech({
    text: script,
    voice: voice,
    language: 'en-US',
    speed: 1.0
  })
}

/**
 * Generate complete course video with all features
 */
export async function generateCompleteCourseVideo(
  videoPrompt: string,
  narrationScript: string,
  options: {
    duration?: number
    resolution?: '720p' | '1080p' | '4K'
    withSoundEffects?: boolean
    instructorVoice?: 'default' | 'male' | 'female'
  } = {}
) {
  return await geminiCompleteService.generateCompleteCourseVideo(
    videoPrompt,
    {
      duration: options.duration || 30,
      resolution: options.resolution || '1080p',
      withSoundEffects: options.withSoundEffects || false,
      withInstructorVoice: true,
      instructorText: narrationScript
    }
  )
}
