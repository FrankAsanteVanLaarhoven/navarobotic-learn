# 🚀 Gemini API Quick Start

## ✅ One API Key, All Features

Your platform now supports **ALL Gemini models** from Google AI Studio with a single API key!

## 🔑 Setup (30 seconds)

1. **Get API Key**: https://aistudio.google.com/app/apikey
2. **Add to `.env`**:
   ```env
   GEMINI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   ```
3. **Done!** 🎉

## 🎯 What You Can Generate

### 1. 📝 Text (Gemini 3 Flash)
```typescript
import { geminiCompleteService } from '@/lib/ai/gemini-complete-service'

await geminiCompleteService.generateText('Create course content')
```

### 2. 🖼️ Images (Nano Banana Pro)
```typescript
await geminiCompleteService.generateImage({
  prompt: 'Course thumbnail',
  model: 'nano-banana-pro'
})
```

### 3. 🎬 Videos (Veo 3.1 with Sound!)
```typescript
await geminiCompleteService.generateVideo({
  prompt: 'Educational video',
  model: 'veo-3.1',
  withSoundEffects: true // 🎵 Sound effects enabled!
})
```

### 4. 🎤 Speech (Gemini TTS)
```typescript
await geminiCompleteService.generateSpeech({
  text: 'Welcome to the course...',
  voice: 'default'
})
```

## 🎬 Generate Course Videos

```bash
# Generate videos for Python Robotics course
bun run generate:python-videos
```

## 📚 Full Documentation

See `GEMINI_COMPLETE_GUIDE.md` for detailed examples and API reference.

---

**One key. All models. Unlimited possibilities.** 🚀
