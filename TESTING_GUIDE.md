# 🧪 Testing Guide for Enhanced Video Generation

Complete guide for testing the enhanced video generation system.

## ✅ System Test Results

The system has been tested and all components are working:

```
✅ Robot Templates - Working
✅ Prompt Generation - Working
✅ Video Configuration - Working
✅ AvatarForcing Integration - Working
✅ NeoVerse 4D Integration - Working
✅ Database Queries - Working
✅ Template Types - Working
```

## 🚀 Quick Start Testing

### 1. Test System Components

```bash
bun run scripts/test-enhanced-video-system.ts
```

This will test all components without making API calls.

### 2. Test API Endpoints (Requires Dev Server)

```bash
# Terminal 1: Start dev server
bun run dev

# Terminal 2: Test API
bun run scripts/test-api-endpoints.ts
```

### 3. Generate Videos for Courses

```bash
# Make sure you have Gemini API credentials in .env
bun run generate:enhanced-videos
```

## 📋 Testing Checklist

### ✅ Component Tests

- [x] Robot templates load correctly
- [x] Prompt generation works
- [x] Video config creation works
- [x] AvatarForcing integration works
- [x] NeoVerse integration works
- [x] Database queries work

### 🔄 API Endpoint Tests

- [ ] POST /api/videos/generate-enhanced (Unitree G1)
- [ ] POST /api/videos/generate-enhanced (Kabuki2)
- [ ] POST /api/videos/generate-enhanced (Custom config)
- [ ] GET /api/videos/generate-enhanced?taskId=...&videoId=...
- [ ] Video status polling
- [ ] Error handling

### 🎬 Video Generation Tests

- [ ] Generate single video
- [ ] Generate videos for all courses
- [ ] Test different robot types
- [ ] Test different template types
- [ ] Test with/without NeoVerse
- [ ] Test with/without AvatarForcing
- [ ] Test different quality settings

### 🎮 Simulation Tests

- [ ] NeoVerse toggle works
- [ ] Simulation renders correctly
- [ ] Camera tracking works
- [ ] Temporal consistency works
- [ ] Physics simulation works

## 🧪 Manual Testing Steps

### Test 1: Generate Unitree G1 Video

```bash
curl -X POST http://localhost:3000/api/videos/generate-enhanced \
  -H "Content-Type: application/json" \
  -d '{
    "lessonId": "lesson-g1-1-1",
    "robotType": "unitree-g1",
    "useNeoVerse": true,
    "useAvatarForcing": true,
    "useVEO3": true,
    "quality": "high",
    "duration": 30,
    "resolution": "1080p"
  }'
```

Expected response:
```json
{
  "success": true,
  "taskId": "...",
  "videoId": "...",
  "status": "processing",
  "metadata": {
    "model": "VEO3 + NeoVerse + AvatarForcing",
    "duration": 30,
    "resolution": "1080p",
    "features": ["4D World Modeling", "Interactive Instructor Avatar", "VEO3 Video Generation"]
  }
}
```

### Test 2: Check Video Status

```bash
curl "http://localhost:3000/api/videos/generate-enhanced?taskId=YOUR_TASK_ID&videoId=YOUR_VIDEO_ID"
```

Expected response (processing):
```json
{
  "success": false,
  "error": undefined
}
```

Expected response (completed):
```json
{
  "success": true,
  "videoUrl": "https://...",
  "metadata": {...}
}
```

### Test 3: Generate Kabuki2 Video

```bash
curl -X POST http://localhost:3000/api/videos/generate-enhanced \
  -H "Content-Type: application/json" \
  -d '{
    "lessonId": "lesson-ros2-1-1",
    "robotType": "kabuki2",
    "useNeoVerse": true,
    "useAvatarForcing": true,
    "useVEO3": true,
    "simulationEnvironment": {
      "type": "gazebo",
      "lighting": "natural",
      "background": "lab"
    }
  }'
```

### Test 4: Test NeoVerse in Simulation

1. Navigate to `/simulation`
2. Toggle "NeoVerse 4D" to ON
3. Verify:
   - Simulation renders with enhanced visuals
   - Camera tracking works smoothly
   - Temporal consistency is maintained
   - Status indicator shows "NeoVerse 4D Active"

### Test 5: Generate All Course Videos

```bash
bun run generate:enhanced-videos
```

This will:
- Process all courses
- Generate videos for all lessons
- Use appropriate robot templates
- Update database with video URLs
- Provide detailed summary

## 🐛 Troubleshooting Tests

### Issue: API Returns 500 Error

**Check:**
1. Gemini API credentials in `.env`
2. Database connection
3. Lesson ID exists
4. Server logs for errors

**Solution:**
```bash
# Check .env file
cat .env | grep GEMINI

# Check database
bun run db:push

# Check server logs
tail -f dev.log
```

### Issue: Video Generation Fails

**Check:**
1. API rate limits
2. Network connection
3. Task status
4. Error messages

**Solution:**
```bash
# Check task status
curl "http://localhost:3000/api/videos/status/YOUR_TASK_ID"

# Check database
bun run db:studio
```

### Issue: NeoVerse Not Working

**Check:**
1. Toggle is enabled
2. Component imports correctly
3. Browser console for errors
4. Three.js is loaded

**Solution:**
```bash
# Check component
cat src/components/simulation/NeoVerseSimulation.tsx

# Check browser console
# Open DevTools > Console
```

## 📊 Expected Test Results

### Component Tests
- ✅ All templates load
- ✅ All prompts generate
- ✅ All configs create
- ✅ All integrations work

### API Tests
- ✅ POST requests succeed
- ✅ GET requests return status
- ✅ Videos generate successfully
- ✅ Database updates correctly

### Video Generation
- ✅ Videos are created
- ✅ URLs are stored
- ✅ Lessons are updated
- ✅ Status tracking works

### Simulation
- ✅ NeoVerse toggle works
- ✅ Visuals are enhanced
- ✅ Performance is good
- ✅ No errors in console

## 🎯 Performance Benchmarks

### Video Generation
- **Task Creation**: < 1 second
- **Generation Time**: 1-10 minutes (depending on duration/quality)
- **Status Polling**: Every 10 seconds
- **Database Update**: < 100ms

### Simulation
- **Frame Rate**: 60 FPS
- **NeoVerse Overhead**: < 5% FPS drop
- **Memory Usage**: < 200MB
- **Load Time**: < 2 seconds

## 📝 Test Reports

After running tests, document:
1. Test date and time
2. Components tested
3. Results (pass/fail)
4. Issues found
5. Performance metrics

## 🔄 Continuous Testing

Set up automated testing:
1. Run component tests on changes
2. Test API endpoints in CI/CD
3. Monitor video generation success rate
4. Track performance metrics

---

**Happy Testing! 🧪**
