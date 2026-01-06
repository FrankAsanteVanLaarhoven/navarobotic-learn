# 📹 Video Access Guide

## 🎬 How to Access Generated Videos

### 1. **Via Course Page (Recommended)**

Navigate to any course page to view videos:

```
http://localhost:3000/courses/[course-slug]
```

**Example:**
- Python 3 for Robotics: `http://localhost:3000/courses/python-robotics`
- ROS Basics: `http://localhost:3000/courses/ros-basics`

**Steps:**
1. Go to the course page
2. Click on a lesson in the sidebar
3. Click the "Video" tab
4. The video will play if available

### 2. **Via API Endpoint**

Get course data including videos:

```bash
GET /api/courses/slug/[course-slug]
```

Returns course data with all lessons and their `videoUrl` fields.

### 3. **Direct Video URL**

Each lesson has a `videoUrl` field that can be accessed directly:

```typescript
// From database
const lesson = await prisma.lesson.findUnique({
  where: { id: 'lesson-id' },
  select: { videoUrl: true }
})

// Access video
if (lesson.videoUrl) {
  // Use in <video> tag or player
  <video src={lesson.videoUrl} controls />
}
```

## 🔍 Finding Courses with Videos

### Check Database

```bash
# List all courses with videos
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); p.lesson.findMany({ where: { videoUrl: { not: null } }, include: { module: { include: { course: true } } }, distinct: ['moduleId'] }).then(lessons => { const courses = new Set(lessons.map(l => l.module.course.slug)); console.log('Courses with videos:', Array.from(courses)); p.\$disconnect(); });"
```

### Check Generated Videos

```bash
# List completed generated videos
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); p.generatedVideo.findMany({ where: { status: 'completed', videoUrl: { not: null } }, include: { lesson: { include: { module: { include: { course: true } } } } } }).then(videos => { const byCourse: any = {}; videos.forEach(v => { const slug = v.lesson?.module?.course?.slug || 'unknown'; if (!byCourse[slug]) { byCourse[slug] = []; } byCourse[slug].push(v); }); Object.entries(byCourse).forEach(([slug, vs]: [string, any]) => { console.log(\`\${slug}: \${vs.length} videos\`); }); p.\$disconnect(); });"
```

## ✅ Identifying Real Videos vs Samples

### Real Generated Videos Have:
- ✅ `status: 'completed'` in `GeneratedVideo` table
- ✅ `videoUrl` that's not null/empty
- ✅ URL doesn't contain: 'sample', 'placeholder', 'example'
- ✅ Created by our generation scripts
- ✅ Associated with a `GeneratedVideo` record

### Sample/Placeholder Videos Have:
- ❌ `videoUrl` contains 'sample', 'placeholder', or 'example'
- ❌ No `GeneratedVideo` record
- ❌ Status is not 'completed'

## 📊 Current Video Status

Check current status:

```bash
# Quick status
bun run monitor:videos

# Detailed check
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); Promise.all([p.generatedVideo.count({ where: { status: 'completed' } }), p.lesson.count({ where: { videoUrl: { not: null } } })]).then(([completed, withUrl]) => { console.log('Completed videos:', completed); console.log('Lessons with URLs:', withUrl); p.\$disconnect(); });"
```

## 🎯 Frontend Integration

Videos are displayed in the course page at:

**File:** `src/app/courses/[slug]/page.tsx`

**Video Tab:**
```tsx
<TabsContent value="video">
  {selectedLesson.videoUrl ? (
    <video src={selectedLesson.videoUrl} controls />
  ) : (
    <div>Video coming soon</div>
  )}
</TabsContent>
```

## 🔗 Direct Links

Once you know the course slug, you can access:

1. **Course Page:** `http://localhost:3000/courses/[slug]`
2. **API:** `http://localhost:3000/api/courses/slug/[slug]`
3. **Video URL:** Direct from `lesson.videoUrl` field

## 📝 Notes

- Videos are stored in the `videoUrl` field of the `Lesson` model
- Generated videos are tracked in the `GeneratedVideo` model
- Videos are accessible via the course page UI
- API endpoints provide programmatic access

---

**🎬 Videos are accessible through the course pages once generated!**
