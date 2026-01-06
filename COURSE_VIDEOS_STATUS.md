# 📹 Course Videos Status & Access Guide

## 🎬 Current Status

**Videos are currently being generated!**

- ⏳ **59 videos** are processing
- ✅ **0 videos** completed (still generating)
- 📊 **Total started:** 59 videos
- 📚 **15/1,143 lessons** have video URLs (1.3% progress)

## 🔍 How to Check Which Courses Have Videos

### Method 1: Check Database

```bash
# List courses with video URLs
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); p.lesson.findMany({ where: { videoUrl: { not: null } }, select: { title: true, videoUrl: true, module: { select: { course: { select: { title: true, slug: true } } } } }, take: 50 }).then(lessons => { const courses = new Set(lessons.map(l => l.module.course.slug)); console.log('Courses with videos:', Array.from(courses)); p.\$disconnect(); });"
```

### Method 2: Check Generated Videos

```bash
# List completed generated videos
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); p.generatedVideo.findMany({ where: { status: 'completed' }, include: { lesson: { include: { module: { include: { course: { select: { title: true, slug: true } } } } } } } }).then(videos => { const courses = new Set(videos.map(v => v.lesson?.module?.course?.slug).filter(Boolean)); console.log('Courses with completed videos:', Array.from(courses)); p.\$disconnect(); });"
```

### Method 3: Real-time Monitor

```bash
bun run monitor:videos
```

## 🚀 How to Access Videos

### 1. **Via Course Page (Web UI)**

Navigate to any course page:

```
http://localhost:3000/courses/[course-slug]
```

**Steps:**
1. Start your dev server: `bun run dev`
2. Go to `http://localhost:3000/courses/[course-slug]`
3. Click on a lesson in the sidebar
4. Click the "Video" tab
5. Video will play if available

**Example URLs:**
- Python 3 for Robotics: `http://localhost:3000/courses/python-robotics`
- ROS Basics: `http://localhost:3000/courses/ros-basics-5-days-python`

### 2. **Via API Endpoint**

```bash
# Get course with videos
curl http://localhost:3000/api/courses/slug/[course-slug]
```

Returns JSON with all lessons and their `videoUrl` fields.

### 3. **Direct Database Query**

```typescript
const lesson = await prisma.lesson.findUnique({
  where: { id: 'lesson-id' },
  select: { videoUrl: true, title: true }
})

// Use video URL
if (lesson.videoUrl) {
  // Not a sample/placeholder
  console.log('Real video:', lesson.videoUrl)
}
```

## ✅ Identifying Real Videos vs Samples

### Real Generated Videos:
- ✅ Have `GeneratedVideo` record with `status: 'completed'`
- ✅ `videoUrl` is not null/empty
- ✅ URL doesn't contain: 'sample', 'placeholder', 'example'
- ✅ Created by our generation scripts
- ✅ Associated with a lesson

### Sample/Placeholder Videos:
- ❌ `videoUrl` contains 'sample', 'placeholder', or 'example'
- ❌ No `GeneratedVideo` record
- ❌ Status is not 'completed'

## 📊 Current Processing Status

**Videos are being generated using:**
- ✅ Veo 3.1 - Video generation (with sound effects)
- ✅ Gemini 3 Flash - Text enhancement
- ✅ Gemini TTS - Instructor narration
- ✅ NeoVerse 4D - World modeling
- ✅ AvatarForcing - Interactive avatars

**Estimated completion:** ~5-6 hours for all 1,143 lessons

## 🔄 Check Progress

```bash
# Quick status
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); Promise.all([p.generatedVideo.count({ where: { status: 'completed' } }), p.generatedVideo.count({ where: { status: 'processing' } }), p.lesson.count({ where: { videoUrl: { not: null } } })]).then(([c, p, l]) => { console.log('Completed:', c); console.log('Processing:', p); console.log('Lessons with URLs:', l); p.\$disconnect(); });"

# Real-time monitor
bun run monitor:videos
```

## 📝 Notes

- Videos are stored in `lesson.videoUrl` field
- Generated videos tracked in `GeneratedVideo` table
- Videos accessible via course pages once generated
- API provides programmatic access
- Frontend displays videos in course lesson pages

## 🎯 Next Steps

1. **Wait for videos to complete** (monitor with `bun run monitor:videos`)
2. **Check which courses have videos** (use queries above)
3. **Access via course pages** (`/courses/[slug]`)
4. **Use API for programmatic access** (`/api/courses/slug/[slug]`)

---

**⏳ Videos are processing! Check back soon or monitor in real-time! 🚀**
