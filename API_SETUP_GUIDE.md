# 🔑 API Setup Guide - Gemini & GLM-4.7

## Overview

This guide shows you how to set up Gemini API and GLM-4.7 for world-class video generation.

## 📋 Required API Keys

### 1. Gemini API (Google AI)
- **Purpose**: Video generation with VEO3
- **Get Key**: https://aistudio.google.com/app/apikey
- **Environment Variable**: `GEMINI_API_KEY` or `GOOGLE_AI_API_KEY`

### 2. GLM-4.7 API
- **Purpose**: Enhanced prompt generation and AI features
- **Get Key**: 
  - Novita AI: https://novita.ai/
  - Glama Gateway: https://glama.ai/
- **Environment Variables**: `GLM_API_KEY`, `GLM47_API_KEY`, or `NOVITA_API_KEY`

### 3. Nanobanana (Optional)
- **Purpose**: Additional API wrapper if needed
- **Environment Variables**: `NANOBANANA_API_KEY`, `NANOBANANA_BASE_URL`

## 🔧 Setup Steps

### Step 1: Create .env File

Copy `.env.example` to `.env`:

```bash
cp .env.example .env
```

### Step 2: Add Your API Keys

Edit `.env` and add your actual API keys:

```env
# Gemini API (Required for video generation)
GEMINI_API_KEY=your_actual_gemini_api_key_here
GOOGLE_AI_API_KEY=your_actual_google_ai_api_key_here

# GLM-4.7 API (Required for prompt enhancement)
GLM_API_KEY=your_actual_glm_api_key_here
# OR
NOVITA_API_KEY=your_actual_novita_api_key_here

# Optional: Nanobanana
NANOBANANA_API_KEY=your_nanobanana_key_here
NANOBANANA_BASE_URL=https://api.nanobanana.com
```

### Step 3: Get Gemini API Key

1. Go to https://aistudio.google.com/app/apikey
2. Sign in with your Google account
3. Click "Create API Key"
4. Copy the key and add it to `.env`

### Step 4: Get GLM-4.7 API Key

**Option A: Novita AI**
1. Go to https://novita.ai/
2. Sign up for an account
3. Navigate to API Keys section
4. Create a new API key
5. Add to `.env` as `NOVITA_API_KEY`

**Option B: Glama Gateway**
1. Go to https://glama.ai/gateway
2. Sign up for an account
3. Get your API key
4. Add to `.env` as `GLM_API_KEY`

### Step 5: Verify Setup

Test the configuration:

```bash
# Check if environment variables are loaded
bun run scripts/test-enhanced-video-system.ts
```

## 🎬 Using the Services

### Generate Videos with Gemini

The system automatically uses Gemini API when configured:

```typescript
import { enhancedVideoService } from '@/lib/video-generation/enhanced-video-service'

const config = {
  lessonId: 'lesson-id',
  robotType: 'unitree-g1',
  useNeoVerse: true,
  useAvatarForcing: true,
  useVEO3: true,
  quality: 'high',
  duration: 30,
  resolution: '1080p'
}

const result = await enhancedVideoService.generateVideo(config)
```

### Enhance Prompts with GLM-4.7

GLM-4.7 automatically enhances video prompts:

```typescript
import { glm47Service } from '@/lib/ai/glm47-service'

const enhancedPrompt = await glm47Service.enhanceVideoPrompt(
  basePrompt,
  {
    lessonTitle: 'Python 3 for Robotics',
    lessonContent: 'Learn Python...',
    robotType: 'generic'
  }
)
```

## 🔒 Security Best Practices

1. **Never commit .env file**
   - Already in `.gitignore`
   - Use `.env.example` for documentation

2. **Use different keys for development/production**
   - Development: Local `.env`
   - Production: Environment variables on server

3. **Rotate keys regularly**
   - Update keys every 90 days
   - Revoke old keys when rotating

4. **Monitor API usage**
   - Check usage in Google AI Studio
   - Monitor Novita/Glama dashboards

## 📊 API Usage

### Gemini API
- **Model**: `gemini-2.0-flash-exp` or `gemini-pro-vision`
- **Video**: VEO3 via Gemini API
- **Rate Limits**: Check Google AI documentation
- **Cost**: Pay-per-use, check pricing

### GLM-4.7 API
- **Model**: `zai-org/glm-4.7`
- **Endpoint**: `https://api.novita.ai/openai/v1/chat/completions`
- **Rate Limits**: Check Novita/Glama documentation
- **Cost**: Pay-per-token, check pricing

## 🐛 Troubleshooting

### Error: "API key not configured"
- Check `.env` file exists
- Verify API key is correct
- Restart development server after adding keys

### Error: "Invalid API key"
- Verify key is correct
- Check key hasn't expired
- Ensure no extra spaces in `.env`

### Error: "Rate limit exceeded"
- Wait before retrying
- Check API usage dashboard
- Consider upgrading API plan

### Error: "Model not found"
- Verify model name is correct
- Check API supports the model
- Update to latest API version

## 📝 Example .env File

```env
# Database
DATABASE_URL="file:./db/custom.db"

# Gemini API (Required)
GEMINI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
GOOGLE_AI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

# GLM-4.7 API (Required)
NOVITA_API_KEY=novita_XXXXXXXXXXXXXXXXXXXXXXXXXXXXX

# Optional
NANOBANANA_API_KEY=your_key_here
NANOBANANA_BASE_URL=https://api.nanobanana.com

# Video Settings
VIDEO_GENERATION_MODEL=gemini-veo3
VIDEO_QUALITY=high
VIDEO_RESOLUTION=1080p
```

## ✅ Verification

After setup, test with:

```bash
# Test video generation
bun run generate:python-videos

# Test system components
bun run scripts/test-enhanced-video-system.ts
```

## 🚀 Next Steps

1. ✅ Add API keys to `.env`
2. ✅ Test configuration
3. ✅ Generate videos: `bun run generate:python-videos`
4. ✅ Monitor API usage
5. ✅ Deploy to production

---

**Ready to generate world-class videos! 🎬**
