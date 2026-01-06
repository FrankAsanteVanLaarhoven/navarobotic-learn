/**
 * Test API Endpoints for Enhanced Video Generation
 * Examples of how to test the API endpoints
 */

// Example 1: Generate Enhanced Video
async function testGenerateEnhancedVideo() {
  const response = await fetch('http://localhost:3000/api/videos/generate-enhanced', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      lessonId: 'lesson-g1-1-1',
      robotType: 'unitree-g1',
      useNeoVerse: true,
      useAvatarForcing: true,
      useVEO3: true,
      quality: 'high',
      duration: 30,
      resolution: '1080p',
      instructorAvatar: {
        name: 'Dr. Sarah Chen',
        style: 'technical',
        voiceProfile: 'voice-bella'
      },
      simulationEnvironment: {
        type: 'hybrid',
        lighting: 'studio',
        background: 'lab'
      }
    })
  })

  const result = await response.json()
  console.log('✅ Video Generation Started:', result)
  return result
}

// Example 2: Generate Video for Kabuki2
async function testGenerateKabuki2Video() {
  const response = await fetch('http://localhost:3000/api/videos/generate-enhanced', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      lessonId: 'lesson-ros2-1-1', // Example lesson ID
      robotType: 'kabuki2',
      useNeoVerse: true,
      useAvatarForcing: true,
      useVEO3: true,
      quality: 'high',
      duration: 30,
      resolution: '1080p',
      simulationEnvironment: {
        type: 'gazebo',
        lighting: 'natural',
        background: 'lab'
      }
    })
  })

  const result = await response.json()
  console.log('✅ Kabuki2 Video Generation Started:', result)
  return result
}

// Example 3: Check Video Status
async function testCheckVideoStatus(taskId: string, videoId: string) {
  const response = await fetch(
    `http://localhost:3000/api/videos/generate-enhanced?taskId=${taskId}&videoId=${videoId}`,
    {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      }
    }
  )

  const result = await response.json()
  console.log('✅ Video Status:', result)
  return result
}

// Example 4: Generate Video with Custom Configuration
async function testCustomConfiguration() {
  const response = await fetch('http://localhost:3000/api/videos/generate-enhanced', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      lessonId: 'lesson-g1-2-1',
      robotType: 'unitree-g1',
      useNeoVerse: true,
      useAvatarForcing: false, // No avatar
      useVEO3: true,
      quality: 'speed', // Faster generation
      duration: 15, // Shorter video
      resolution: '720p', // Lower resolution
      simulationEnvironment: {
        type: 'real-world',
        lighting: 'natural',
        background: 'outdoor'
      }
    })
  })

  const result = await response.json()
  console.log('✅ Custom Configuration Video:', result)
  return result
}

// Example 5: Poll for Video Completion
async function pollVideoCompletion(taskId: string, videoId: string, maxAttempts: number = 60) {
  console.log(`⏳ Polling for video completion (max ${maxAttempts} attempts)...`)
  
  for (let i = 0; i < maxAttempts; i++) {
    const status = await testCheckVideoStatus(taskId, videoId)
    
    if (status.success && status.videoUrl) {
      console.log(`✅ Video completed! URL: ${status.videoUrl}`)
      return status
    }
    
    if (status.error) {
      console.log(`❌ Video generation failed: ${status.error}`)
      return status
    }
    
    console.log(`⏳ Attempt ${i + 1}/${maxAttempts}: Still processing...`)
    await new Promise(resolve => setTimeout(resolve, 10000)) // Wait 10 seconds
  }
  
  console.log(`⏸️  Max attempts reached. Video may still be processing.`)
  return null
}

// Main test function
async function main() {
  console.log('🧪 Testing Enhanced Video Generation API Endpoints\n')
  console.log('='.repeat(60))
  
  // Note: These are example calls. Uncomment to test:
  
  // Test 1: Generate Unitree G1 video
  // const result1 = await testGenerateEnhancedVideo()
  // if (result1.success && result1.taskId) {
  //   await pollVideoCompletion(result1.taskId, result1.videoId!)
  // }
  
  // Test 2: Generate Kabuki2 video
  // await testGenerateKabuki2Video()
  
  // Test 3: Custom configuration
  // await testCustomConfiguration()
  
  console.log('\n📝 To test these endpoints:')
  console.log('   1. Start the dev server: bun run dev')
  console.log('   2. Uncomment the test calls above')
  console.log('   3. Run: bun run scripts/test-api-endpoints.ts')
  console.log('\n💡 Or use curl/Postman:')
  console.log('   curl -X POST http://localhost:3000/api/videos/generate-enhanced \\')
  console.log('     -H "Content-Type: application/json" \\')
  console.log('     -d \'{"lessonId":"lesson-g1-1-1","robotType":"unitree-g1","useNeoVerse":true}\'')
}

// Export for use in other scripts
export {
  testGenerateEnhancedVideo,
  testGenerateKabuki2Video,
  testCheckVideoStatus,
  testCustomConfiguration,
  pollVideoCompletion
}

// Run if executed directly
if (import.meta.main) {
  main().catch(console.error)
}
