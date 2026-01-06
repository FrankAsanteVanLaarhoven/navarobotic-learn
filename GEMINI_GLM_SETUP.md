# 🔑 Gemini API & GLM-4.7 Setup Guide

## ✅ What's Been Configured

I've integrated **Gemini API** and **GLM-4.7** for world-class video generation:

1. ✅ **Gemini Video Service** - Uses Google's Gemini API with VEO3
2. ✅ **GLM-4.7 Service** - Enhances video prompts with AI
3. ✅ **Enhanced Video Service** - Integrated both services
4. ✅ **.env Configuration** - Ready for your API keys

## 🔧 Setup Steps

### Step 1: Get Gemini API Key

1. Go to **https://aistudio.google.com/app/apikey**
2. Sign in with your Google account
3. Click **"Create API Key"**
4. Copy the key (starts with `AIza...`)

### Step 2: Get GLM-4.7 API Key

**Option A: Novita AI (Recommended)**
1. Go to **https://novita.ai/**
2. Sign up for an account
3. Navigate to **API Keys** section
4. Create a new API key
5. Copy the key

**Option B: Glama Gateway**
1. Go to **https://glama.ai/gateway**
2. Sign up for an account
3. Get your API key from dashboard

### Step 3: Configure .env File

Edit `.env` file in project root:

```env
# Gemini API (Required for video generation)
GEMINI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
GOOGLE_AI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

# GLM-4.7 API (Required for prompt enhancement)
NOVITA_API_KEY=novita_XXXXXXXXXXXXXXXXXXXXXXXXXXXXX
# OR
GLM_API_KEY=your_glm_api_key_here
GLM47_API_KEY=your_glm47_api_key_here

# Optional: Nanobanana
NANOBANANA_API_KEY=your_nanobanana_key_here
NANOBANANA_BASE_URL=https://api.nanobanana.com
```

### Step 4: Verify Setup

Test the configuration:

```bash
# Check if services are configured
bun run scripts/test-enhanced-video-system.ts
```

## 🎬 Generate Videos

Once configured, generate videos:

```bash
# Generate Python 3 for Robotics videos
bun run generate:python-videos

# Generate all course videos
bun run generate:enhanced-videos
```

## 📊 How It Works

### 1. Prompt Enhancement (GLM-4.7)
- Base prompt is enhanced with GLM-4.7
- Adds comprehensive explanations
- Improves video generation quality

### 2. Video Generation (Gemini VEO3)
- Uses Gemini API with VEO3 model
- Generates high-quality educational videos
- Includes NeoVerse 4D and AvatarForcing features

### 3. Integration
- Both services work together seamlessly
- GLM-4.7 enhances prompts
- Gemini generates videos
- All automated

## 🔒 Security

- ✅ `.env` file is in `.gitignore`
- ✅ Never commit API keys
- ✅ Use `.env.example` for documentation
- ✅ Rotate keys regularly

## 🐛 Troubleshooting

### "Gemini API key not configured"
- Check `.env` file exists
- Verify `GEMINI_API_KEY` is set
- Restart dev server after adding keys

### "GLM-4.7 API key not configured"
- Check `NOVITA_API_KEY` or `GLM_API_KEY` is set
- Verify key is correct
- Check API service is accessible

### API Errors
- Verify keys are correct
- Check API quotas/limits
- Ensure internet connection

## 📝 Example .env

```env
# Database
DATABASE_URL="file:./db/custom.db"

# Gemini API (Required)
GEMINI_API_KEY=AIzaSyAbCdEfGhIjKlMnOpQrStUvWxYz1234567890
GOOGLE_AI_API_KEY=AIzaSyAbCdEfGhIjKlMnOpQrStUvWxYz1234567890

# GLM-4.7 API (Required)
NOVITA_API_KEY=novita_abcdefghijklmnopqrstuvwxyz1234567890

# Video Settings
VIDEO_GENERATION_MODEL=gemini-veo3
VIDEO_QUALITY=high
VIDEO_RESOLUTION=1080p
```

## ✅ Verification

After setup, test:

```bash
# Test system
bun run scripts/test-enhanced-video-system.ts

# Generate test video
bun run generate:python-videos
```

---

**Ready to generate world-class videos with Gemini & GLM-4.7! 🚀**
