# Video Generation Scripts

## Generate Course Videos

Generate AI-powered educational videos for all lessons in the Unitree G1 Fundamentals course using Gemini 2.5 with Veo 3.

### Prerequisites

1. **Gemini API Key**: Set up your Gemini API credentials (supports Veo 3.1 with sound effects)
2. **Environment Variables**: Ensure your `.env` file has the necessary API keys

### Usage

```bash
# Generate videos for all 12 lessons
bun run generate:videos
```

### What It Does

1. ✅ Fetches all lessons from Unitree G1 Fundamentals course
2. ✅ Generates optimized prompts for each lesson
3. ✅ Creates video generation tasks using Veo 3 (Gemini 2.5)
4. ✅ Polls for completion (up to 10 minutes per video)
5. ✅ Updates database with video URLs automatically
6. ✅ Provides detailed summary of results

### Video Specifications

- **Model**: Veo 3 (via Gemini 2.5)
- **Quality**: High
- **Resolution**: 1920x1080 (Full HD)
- **FPS**: 30
- **Duration**: 30 seconds per lesson
- **Audio**: Can be enabled for synchronized narration

### Output

The script will:
- Show progress for each lesson
- Update the database with video URLs
- Display a summary of successful/failed generations
- Save all video URLs to the lessons table

### Troubleshooting

If videos fail to generate:
1. Check API credentials
2. Verify internet connection
3. Check API rate limits
4. Review error messages in console
5. Try generating individual lessons
