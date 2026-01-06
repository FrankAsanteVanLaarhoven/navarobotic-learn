# 🎬 Video Generation Guide for Unitree G1 Fundamentals Course

## Overview

This guide explains how to generate AI-powered educational videos for all 12 lessons in the Unitree G1 Fundamentals course using **Gemini 2.5 with Veo 3** for state-of-the-art video generation.

## 🚀 Quick Start

### Option 1: Command Line Script (Recommended)

Run the automated script to generate videos for all lessons:

```bash
bun run generate:videos
```

This script will:
- ✅ Fetch all 12 lessons from the Unitree G1 Fundamentals course
- ✅ Generate optimized prompts for each lesson
- ✅ Create video generation tasks using ZAI SDK
- ✅ Poll for completion and update the database automatically
- ✅ Provide a detailed summary of results

### Option 2: API Endpoints

Generate videos programmatically via API:

#### Generate Video for a Lesson

```bash
POST /api/videos/generate
Content-Type: application/json

{
  "lessonId": "lesson-g1-1-1",
  "model": "high"  // or "speed"
}
```

#### Check Video Generation Status

```bash
GET /api/videos/status/[taskId]
```

## 📋 Lesson Prompts

Each lesson has a carefully crafted prompt optimized for educational video generation:

### Module 1: Introduction to Unitree G1
- **Lesson 1.1**: Robot overview with technical specifications
- **Lesson 1.2**: Development environment setup tutorial
- **Lesson 1.3**: Safety and basic control demonstrations

### Module 2: Walking Control Systems
- **Lesson 2.1**: Bipedal locomotion fundamentals with animations
- **Lesson 2.2**: Balance control algorithms and PID controllers
- **Lesson 2.3**: Gait generation and walking patterns

### Module 3: Navigation and Path Planning
- **Lesson 3.1**: Path planning algorithms (A*, RRT)
- **Lesson 3.2**: Obstacle avoidance with LiDAR
- **Lesson 3.3**: SLAM and localization systems

### Module 4: Perception Systems
- **Lesson 4.1**: Vision systems and camera integration
- **Lesson 4.2**: Sensor fusion techniques
- **Lesson 4.3**: Environmental understanding and scene analysis

## ⚙️ Configuration

### Video Settings

- **Quality**: High (for educational content)
- **Resolution**: 1920x1080 (Full HD)
- **FPS**: 30
- **Duration**: 30 seconds per video
- **Audio**: Disabled (can be added separately with TTS)

### Environment Variables

Make sure you have Google AI API credentials configured:

```bash
# .env file
GOOGLE_AI_API_KEY=your_google_ai_api_key_here
```

**Note**: The system uses Gemini 2.5 with Veo 3 integration for video generation. Veo 3 provides:
- 8-second video clips at 1080p resolution
- Synchronized audio support
- High-quality educational content generation
- Professional cinematography suitable for learning platforms

## 📊 Generation Process

1. **Task Creation**: Each lesson creates a video generation task
2. **Polling**: Script polls every 10 seconds for completion
3. **Database Update**: Video URLs are automatically saved to lessons
4. **Summary**: Detailed report of successful/failed generations

## 🎯 Best Practices

1. **Rate Limiting**: Script includes 5-second delays between requests
2. **Error Handling**: Failed generations are logged but don't stop the process
3. **Retry Logic**: You can re-run the script for failed lessons
4. **Quality**: Use "high" quality for final videos, "speed" for testing

## 🔄 Updating Existing Videos

To regenerate a video for a specific lesson:

```typescript
// Update the prompt in scripts/generate-course-videos.ts
// Then run:
bun run generate:videos
```

Or use the API:

```bash
POST /api/videos/generate
{
  "lessonId": "lesson-g1-1-1"
}
```

## 📝 Custom Prompts

To customize prompts for specific lessons, edit the `lessonPrompts` object in:
- `scripts/generate-course-videos.ts` (for CLI)
- `src/app/api/videos/generate/route.ts` (for API)

## 🎨 Video Style Guidelines

All prompts follow these guidelines:
- Professional educational presentation style
- Clean studio or lab settings
- Technical diagrams and annotations
- Clear, step-by-step demonstrations
- Realistic robot movements and behaviors

## ⚠️ Notes

- Video generation can take 1-10 minutes per video
- Some videos may require multiple attempts
- Check the console output for detailed progress
- Failed videos can be regenerated individually

## 🎉 Success Indicators

After running the script, you should see:
- ✅ All 12 lessons with video URLs in the database
- ✅ Videos playable in the course page
- ✅ No "Video coming soon" placeholders

## 📞 Support

If videos fail to generate:
1. Check ZAI API credentials
2. Verify internet connection
3. Check API rate limits
4. Review error messages in console
5. Try generating individual lessons via API
