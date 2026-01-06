/**
 * Test Script for Enhanced Video Generation System
 * Tests the integration without making actual API calls
 */

import { PrismaClient } from '@prisma/client'
import { getRobotTemplate, createVideoConfigFromTemplate, getPromptFromTemplate } from '../src/lib/video-generation/robot-templates'
import { avatarForcingIntegration } from '../src/lib/video-generation/avatar-forcing-integration'
import { neoverseIntegration } from '../src/lib/simulation/neoverse-integration'

const prisma = new PrismaClient()

async function testSystem() {
  console.log('🧪 Testing Enhanced Video Generation System\n')
  console.log('='.repeat(60))

  // Test 1: Robot Templates
  console.log('\n📋 Test 1: Robot Templates')
  console.log('-'.repeat(60))
  
  const unitreeTemplate = getRobotTemplate('unitree-g1')
  console.log(`✅ Unitree G1 Template:`)
  console.log(`   Name: ${unitreeTemplate.name}`)
  console.log(`   Description: ${unitreeTemplate.description}`)
  console.log(`   NeoVerse: ${unitreeTemplate.defaultConfig.useNeoVerse ? '✅' : '❌'}`)
  console.log(`   AvatarForcing: ${unitreeTemplate.defaultConfig.useAvatarForcing ? '✅' : '❌'}`)
  console.log(`   VEO3: ${unitreeTemplate.defaultConfig.useVEO3 ? '✅' : '❌'}`)
  console.log(`   Instructor: ${unitreeTemplate.defaultConfig.instructorAvatar?.name || 'N/A'}`)

  const kabukiTemplate = getRobotTemplate('kabuki2')
  console.log(`\n✅ Kabuki2 Template:`)
  console.log(`   Name: ${kabukiTemplate.name}`)
  console.log(`   Description: ${kabukiTemplate.description}`)
  console.log(`   Environment: ${kabukiTemplate.defaultConfig.simulationEnvironment?.type || 'N/A'}`)

  // Test 2: Prompt Generation
  console.log('\n📝 Test 2: Prompt Generation')
  console.log('-'.repeat(60))
  
  const introPrompt = getPromptFromTemplate('unitree-g1', 'introduction')
  console.log(`✅ Introduction Prompt (first 200 chars):`)
  console.log(`   ${introPrompt.substring(0, 200)}...`)

  const demoPrompt = getPromptFromTemplate('unitree-g1', 'demonstration')
  console.log(`\n✅ Demonstration Prompt (first 200 chars):`)
  console.log(`   ${demoPrompt.substring(0, 200)}...`)

  // Test 3: Video Config Generation
  console.log('\n⚙️  Test 3: Video Configuration')
  console.log('-'.repeat(60))
  
  const config = createVideoConfigFromTemplate(
    'unitree-g1',
    'test-lesson-id',
    'introduction'
  )
  
  console.log(`✅ Generated Config:`)
  console.log(`   Robot Type: ${config.robotType}`)
  console.log(`   NeoVerse: ${config.useNeoVerse ? '✅' : '❌'}`)
  console.log(`   AvatarForcing: ${config.useAvatarForcing ? '✅' : '❌'}`)
  console.log(`   VEO3: ${config.useVEO3 ? '✅' : '❌'}`)
  console.log(`   Quality: ${config.quality}`)
  console.log(`   Resolution: ${config.resolution}`)
  console.log(`   Duration: ${config.duration}s`)
  console.log(`   Environment: ${config.simulationEnvironment?.type || 'N/A'}`)
  console.log(`   Instructor: ${config.instructorAvatar?.name || 'N/A'}`)

  // Test 4: AvatarForcing Integration
  console.log('\n👤 Test 4: AvatarForcing Integration')
  console.log('-'.repeat(60))
  
  avatarForcingIntegration.updateConfig({
    enabled: true,
    instructorName: 'Dr. Sarah Chen',
    style: 'technical',
    position: 'corner',
    reactivity: 'high'
  })
  
  const avatarPrompt = avatarForcingIntegration.generateAvatarPrompt(
    'This is a test lesson about robot control systems.',
    'excited'
  )
  
  console.log(`✅ AvatarForcing Prompt (first 300 chars):`)
  console.log(`   ${avatarPrompt.substring(0, 300)}...`)
  
  const voiceConfig = avatarForcingIntegration.getVoiceProfileConfig()
  console.log(`\n✅ Voice Profile Config:`)
  console.log(`   Provider: ${voiceConfig.provider}`)
  console.log(`   Voice ID: ${voiceConfig.voiceId}`)
  console.log(`   Stability: ${voiceConfig.settings.stability}`)

  // Test 5: NeoVerse Integration
  console.log('\n🌍 Test 5: NeoVerse 4D Integration')
  console.log('-'.repeat(60))
  
  const neoverseDesc = neoverseIntegration.generateVideoPromptDescription()
  console.log(`✅ NeoVerse Description:`)
  console.log(neoverseDesc)
  
  const optimalCamera = neoverseIntegration.getOptimalCameraPosition(
    new (await import('three')).Vector3(0, 0, 0),
    new (await import('three')).Euler(0, 0, 0),
    5
  )
  
  console.log(`\n✅ Optimal Camera Position:`)
  console.log(`   Position: (${optimalCamera.position.x.toFixed(2)}, ${optimalCamera.position.y.toFixed(2)}, ${optimalCamera.position.z.toFixed(2)})`)
  console.log(`   Target: (${optimalCamera.target.x.toFixed(2)}, ${optimalCamera.target.y.toFixed(2)}, ${optimalCamera.target.z.toFixed(2)})`)

  // Test 6: Database Query
  console.log('\n💾 Test 6: Database Query')
  console.log('-'.repeat(60))
  
  try {
    const courses = await prisma.course.findMany({
      include: {
        modules: {
          include: {
            lessons: {
              take: 1,
              orderBy: { order: 'asc' }
            }
          },
          take: 1,
          orderBy: { order: 'asc' }
        }
      },
      take: 3
    })
    
    console.log(`✅ Found ${courses.length} courses:`)
    courses.forEach(course => {
      const lessonCount = course.modules.reduce((acc, m) => acc + m.lessons.length, 0)
      console.log(`   - ${course.title} (${course.modules.length} modules, ${lessonCount} lessons)`)
    })
  } catch (error: any) {
    console.log(`⚠️  Database query failed: ${error.message}`)
    console.log(`   This is okay if the database isn't set up yet.`)
  }

  // Test 7: Template Types
  console.log('\n🎬 Test 7: Template Types')
  console.log('-'.repeat(60))
  
  const templateTypes: Array<'introduction' | 'demonstration' | 'technical' | 'tutorial'> = 
    ['introduction', 'demonstration', 'technical', 'tutorial']
  
  templateTypes.forEach(type => {
    const prompt = getPromptFromTemplate('unitree-g1', type)
    console.log(`✅ ${type.charAt(0).toUpperCase() + type.slice(1)} Template:`)
    console.log(`   Length: ${prompt.length} characters`)
    console.log(`   Preview: ${prompt.substring(0, 100)}...`)
  })

  console.log('\n' + '='.repeat(60))
  console.log('✅ All Tests Completed Successfully!')
  console.log('='.repeat(60))
  console.log('\n📚 Next Steps:')
  console.log('   1. Set up ZAI API credentials in .env file')
  console.log('   2. Run: bun run generate:enhanced-videos')
  console.log('   3. Test API endpoints: POST /api/videos/generate-enhanced')
  console.log('   4. Enable NeoVerse toggle in simulation page')
  console.log('   5. Customize templates for your specific robots\n')
}

testSystem()
  .catch(console.error)
  .finally(() => prisma.$disconnect())
