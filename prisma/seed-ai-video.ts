import { PrismaClient } from '@prisma/client'

const prisma = new PrismaClient()

async function main() {
  console.log('Seeding AI video generation data...')

  // Create Video Generation Models
  await prisma.videoGenerationModel.upsert({
    where: { name: 'sora' },
    update: {},
    create: {
      id: 'model-sora',
      name: 'Sora',
      provider: 'openai',
      description: 'OpenAI\'s text-to-video model with high-quality output',
      capabilities: 'text-to-video',
      maxDuration: 60,
      resolution: '1080p',
      pricing: {
        costPerMinute: 0.20,
        currency: 'USD'
      },
      isActive: true
    }
  })

  await prisma.videoGenerationModel.upsert({
    where: { name: 'kling' },
    update: {},
    create: {
      id: 'model-kling',
      name: 'Kling AI',
      provider: 'kling',
      description: 'High-quality video generation with realistic movements',
      capabilities: 'text-to-video, image-to-video',
      maxDuration: 120,
      resolution: '1080p',
      pricing: {
        costPerMinute: 0.10,
        currency: 'USD'
      },
      isActive: true
    }
  })

  await prisma.videoGenerationModel.upsert({
    where: { name: 'veo' },
    update: {},
    create: {
      id: 'model-veo',
      name: 'Veo',
      provider: 'veo',
      description: 'AI-powered video creation with cinematic quality',
      capabilities: 'text-to-video',
      maxDuration: 60,
      resolution: '4K',
      pricing: {
        costPerMinute: 0.05,
        currency: 'USD'
      },
      isActive: true
    }
  })

  await prisma.videoGenerationModel.upsert({
    where: { name: 'synthara' },
    update: {},
    create: {
      id: 'model-synthara',
      name: 'Synthara',
      provider: 'synthara',
      description: 'Photorealistic AI video generation',
      capabilities: 'text-to-video',
      maxDuration: 180,
      resolution: '4K',
      pricing: {
        costPerMinute: 0.08,
        currency: 'USD'
      },
      isActive: true
    }
  })

  await prisma.videoGenerationModel.upsert({
    where: { name: 'runway' },
    update: {},
    create: {
      id: 'model-runway',
      name: 'Runway',
      provider: 'runwayml',
      description: 'Professional video generation for creators',
      capabilities: 'text-to-video, image-to-video',
      maxDuration: 90,
      resolution: '1080p',
      pricing: {
        costPerMinute: 0.25,
        currency: 'USD'
      },
      isActive: true
    }
  })

  await prisma.videoGenerationModel.upsert({
    where: { name: 'luma' },
    update: {},
    create: {
      id: 'model-luma',
      name: 'Luma',
      provider: 'luma',
      description: 'Realistic 3D video generation',
      capabilities: 'text-to-video',
      maxDuration: 60,
      resolution: '720p',
      pricing: {
        costPerMinute: 0.15,
        currency: 'USD'
      },
      isActive: true
    }
  })

  await prisma.videoGenerationModel.upsert({
    where: { name: 'pika' },
    update: {},
    create: {
      id: 'model-pika',
      name: 'Pika',
      provider: 'pika',
      description: 'Fast AI video generation for creators',
      capabilities: 'text-to-video',
      maxDuration: 120,
      resolution: '1080p',
      pricing: {
        costPerMinute: 0.18,
        currency: 'USD'
      },
      isActive: true
    }
  })

  // Create Voice Profiles (like ElevenLabs)
  await prisma.voiceProfile.upsert({
    where: { name: 'adam' },
    update: {},
    create: {
      id: 'voice-adam',
      name: 'Adam',
      displayName: 'Adam',
      gender: 'male',
      accent: 'american',
      age: 'young',
      style: 'professional',
      provider: 'elevenlabs',
      sampleAudioUrl: '/audio/samples/adam.mp3',
      isActive: true
    }
  })

  await prisma.voiceProfile.upsert({
    where: { name: 'bella' },
    update: {},
    create: {
      id: 'voice-bella',
      name: 'Bella',
      displayName: 'Bella',
      gender: 'female',
      accent: 'american',
      age: 'young',
      style: 'professional',
      provider: 'elevenlabs',
      sampleAudioUrl: '/audio/samples/bella.mp3',
      isActive: true
    }
  })

  await prisma.voiceProfile.upsert({
    where: { name: 'charlie' },
    update: {},
    create: {
      id: 'voice-charlie',
      name: 'Charlie',
      displayName: 'Charlie',
      gender: 'male',
      accent: 'british',
      age: 'middle-aged',
      style: 'casual',
      provider: 'elevenlabs',
      sampleAudioUrl: '/audio/samples/charlie.mp3',
      isActive: true
    }
  })

  await prisma.voiceProfile.upsert({
    where: { name: 'diana' },
    update: {},
    create: {
      id: 'voice-diana',
      name: 'Diana',
      displayName: 'Diana',
      gender: 'female',
      accent: 'australian',
      age: 'middle-aged',
      style: 'excited',
      provider: 'elevenlabs',
      sampleAudioUrl: '/audio/samples/diana.mp3',
      isActive: true
    }
  })

  await prisma.voiceProfile.upsert({
    where: { name: 'ethan' },
    update: {},
    create: {
      id: 'voice-ethan',
      name: 'Ethan',
      displayName: 'Ethan',
      gender: 'male',
      accent: 'american',
      age: 'elderly',
      style: 'calm',
      provider: 'elevenlabs',
      sampleAudioUrl: '/audio/samples/ethan.mp3',
      isActive: true
    }
  })

  await prisma.voiceProfile.upsert({
    where: { name: 'nova' },
    update: {},
    create: {
      id: 'voice-nova',
      name: 'Nova',
      displayName: 'Nova',
      gender: 'neutral',
      accent: 'american',
      age: 'young',
      style: 'professional',
      provider: 'elevenlabs',
      sampleAudioUrl: '/audio/samples/nova.mp3',
      isActive: true
    }
  })

  console.log('✅ Created 7 video generation models')
  console.log('✅ Created 6 voice profiles')
  console.log('✅ Database seeded with AI video generation data!')
}

main()
  .catch((e) => {
    console.error('Error seeding database:', e)
    process.exit(1)
  })
  .finally(async () => {
    await prisma.$disconnect()
  })
