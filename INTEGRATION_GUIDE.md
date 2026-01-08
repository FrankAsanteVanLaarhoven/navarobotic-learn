# 🔌 Backend Integration Guide

## Overview

This guide documents all backend integrations implemented for the NAVA-ROBOTICLEARN platform.

## ✅ Implemented Integrations

### 1. NextAuth.js Authentication

**Status**: ✅ Implemented

**Location**: `src/lib/auth.ts`, `src/app/api/auth/[...nextauth]/route.ts`

**Features**:
- Email/Password authentication
- Google OAuth
- GitHub OAuth
- Session management
- JWT tokens
- Role-based access control

**Setup**:
```bash
# Add to .env
NEXTAUTH_SECRET=your-secret-key-here
NEXTAUTH_URL=http://localhost:3000
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

**Usage**:
```typescript
import { getServerSession } from 'next-auth'
import { authOptions } from '@/lib/auth'

const session = await getServerSession(authOptions)
```

### 2. WebSocket Real-time Features

**Status**: ✅ Implemented

**Location**: `src/lib/websocket.ts`

**Features**:
- Real-time chat
- Video session updates
- Telemetry broadcasting
- Room-based messaging

**Setup**:
```typescript
import { getWebSocketService } from '@/lib/websocket'

const wsService = getWebSocketService()
wsService.initialize(httpServer)
```

**Client Connection**:
```javascript
const ws = new WebSocket('ws://localhost:3000?userId=user123')
ws.send(JSON.stringify({ type: 'chat', payload: { message: 'Hello' } }))
```

### 3. File Storage (AWS S3 / Cloudflare R2)

**Status**: ✅ Implemented

**Location**: `src/lib/storage.ts`, `src/app/api/storage/upload/route.ts`

**Features**:
- File uploads
- Signed URLs
- Public URLs
- File deletion

**Setup**:
```bash
# AWS S3
STORAGE_PROVIDER=s3
STORAGE_ACCESS_KEY_ID=your-access-key
STORAGE_SECRET_ACCESS_KEY=your-secret-key
STORAGE_BUCKET=your-bucket-name
STORAGE_REGION=us-east-1

# Cloudflare R2
STORAGE_PROVIDER=r2
STORAGE_ACCESS_KEY_ID=your-r2-access-key
STORAGE_SECRET_ACCESS_KEY=your-r2-secret-key
STORAGE_BUCKET=your-bucket-name
STORAGE_ENDPOINT=https://your-account-id.r2.cloudflarestorage.com
CLOUDFLARE_ACCOUNT_ID=your-account-id
```

**Usage**:
```typescript
import { getStorageService } from '@/lib/storage'

const storage = getStorageService()
const url = await storage.uploadFile('path/to/file', buffer, 'image/png')
```

### 4. Stripe Payment Processing

**Status**: ✅ Implemented

**Location**: `src/lib/stripe.ts`, `src/app/api/stripe/webhook/route.ts`

**Features**:
- One-time payments
- Subscriptions
- Webhook handling
- Customer management

**Setup**:
```bash
STRIPE_SECRET_KEY=sk_test_...
STRIPE_PUBLISHABLE_KEY=pk_test_...
STRIPE_WEBHOOK_SECRET=whsec_...
```

**Usage**:
```typescript
import { createCheckoutSession } from '@/lib/stripe'

const session = await createCheckoutSession({
  userId: 'user123',
  courseId: 'course123',
  priceId: 'price_123',
  successUrl: 'https://app.com/success',
  cancelUrl: 'https://app.com/cancel'
})
```

### 5. Email Notifications (SendGrid / Resend)

**Status**: ✅ Implemented

**Location**: `src/lib/email.ts`

**Features**:
- Welcome emails
- Course enrollment emails
- Password reset emails
- Course completion emails

**Setup**:
```bash
# SendGrid
EMAIL_PROVIDER=sendgrid
EMAIL_API_KEY=SG.xxx
EMAIL_FROM=noreply@navaroboticlearn.com

# Resend
EMAIL_PROVIDER=resend
EMAIL_API_KEY=re_xxx
EMAIL_FROM=noreply@navaroboticlearn.com
```

**Usage**:
```typescript
import { getEmailService } from '@/lib/email'

const emailService = getEmailService()
await emailService.sendWelcomeEmail('user@example.com', 'John Doe')
```

### 6. Code Execution (Docker / AWS Lambda)

**Status**: ✅ Implemented

**Location**: `src/lib/code-execution.ts`, `src/app/api/code/execute/route.ts`

**Features**:
- Sandboxed code execution
- Multiple language support (Python, JavaScript, C++, Rust)
- Timeout protection
- Memory limits
- Network isolation

**Setup**:
```bash
CODE_EXECUTION_PROVIDER=docker
CODE_EXECUTION_TIMEOUT=30000
CODE_EXECUTION_MEMORY_LIMIT=512m
```

**Usage**:
```typescript
import { getCodeExecutionService } from '@/lib/code-execution'

const codeService = getCodeExecutionService()
const result = await codeService.executeCode({
  code: 'print("Hello, World!")',
  language: 'python'
})
```

### 7. PostgreSQL Database

**Status**: ✅ Migration Guide Ready

**Location**: `POSTGRESQL_MIGRATION.md`

**Features**:
- Production-ready database
- Connection pooling support
- SSL/TLS support
- Backup strategies

**Setup**: See `POSTGRESQL_MIGRATION.md` for detailed instructions.

## ⚠️ Conceptual Integrations

### 8. NeoVerse 4D API

**Status**: ⚠️ Architecture Ready, API Not Available

**Location**: `src/lib/simulation/neoverse-integration.ts`

**Integration Points**:
- `generateNeoVersePrompt()` - Creates 4D world modeling prompts
- `applyNeoVerseConfig()` - Applies NeoVerse settings
- `enhanceWithNeoVerse()` - Enhances video prompts with NeoVerse

**When NeoVerse API is available**:
1. Add API key to `.env`: `NEOVERSE_API_KEY=xxx`
2. Update `neoverse-integration.ts` to call real API
3. Replace simulated features with actual API calls

**Expected API Format**:
```typescript
interface NeoVerseAPI {
  generateWorldModel(prompt: string, config: NeoVerseConfig): Promise<WorldModel>
  enhanceVideo(videoPrompt: string, worldModel: WorldModel): Promise<EnhancedPrompt>
}
```

### 9. AvatarForcing API

**Status**: ⚠️ Architecture Ready, API Not Available

**Location**: `src/lib/video-generation/avatar-forcing-integration.ts`

**Integration Points**:
- `generateAvatarPrompt()` - Creates avatar generation prompts
- `applyAvatarForcing()` - Applies AvatarForcing settings
- `enhanceWithAvatar()` - Enhances video prompts with avatar

**When AvatarForcing API is available**:
1. Add API key to `.env`: `AVATAR_FORCING_API_KEY=xxx`
2. Update `avatar-forcing-integration.ts` to call real API
3. Replace simulated features with actual API calls

**Expected API Format**:
```typescript
interface AvatarForcingAPI {
  generateAvatar(prompt: string, config: AvatarConfig): Promise<AvatarVideo>
  syncLipSync(audio: Buffer, avatar: AvatarVideo): Promise<SyncedVideo>
}
```

## 📋 Environment Variables Template

Create a `.env` file with all required variables:

```bash
# Database
DATABASE_URL="postgresql://user:pass@localhost:5432/navaroboticlearn"

# NextAuth
NEXTAUTH_SECRET=your-secret-key-here
NEXTAUTH_URL=http://localhost:3000
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret

# Storage (AWS S3 or Cloudflare R2)
STORAGE_PROVIDER=s3
STORAGE_ACCESS_KEY_ID=your-access-key
STORAGE_SECRET_ACCESS_KEY=your-secret-key
STORAGE_BUCKET=your-bucket-name
STORAGE_REGION=us-east-1

# Stripe
STRIPE_SECRET_KEY=sk_test_...
STRIPE_PUBLISHABLE_KEY=pk_test_...
STRIPE_WEBHOOK_SECRET=whsec_...

# Email (SendGrid or Resend)
EMAIL_PROVIDER=resend
EMAIL_API_KEY=re_xxx
EMAIL_FROM=noreply@navaroboticlearn.com

# Code Execution
CODE_EXECUTION_PROVIDER=docker
CODE_EXECUTION_TIMEOUT=30000
CODE_EXECUTION_MEMORY_LIMIT=512m

# Gemini API (already configured)
GEMINI_API_KEY=your-gemini-api-key
GOOGLE_AI_API_KEY=your-google-ai-api-key

# NeoVerse (when available)
NEOVERSE_API_KEY=xxx

# AvatarForcing (when available)
AVATAR_FORCING_API_KEY=xxx
```

## 🚀 Next Steps

1. **Configure Environment Variables** - Add all required API keys
2. **Set Up PostgreSQL** - Follow `POSTGRESQL_MIGRATION.md`
3. **Test Integrations** - Use provided API endpoints
4. **Deploy to Production** - Update environment variables for production
5. **Monitor Services** - Set up logging and monitoring

## 📚 API Endpoints

### Authentication
- `POST /api/auth/register` - Register new user
- `GET/POST /api/auth/[...nextauth]` - NextAuth endpoints

### Storage
- `POST /api/storage/upload` - Upload file

### Payments
- `POST /api/stripe/webhook` - Stripe webhook handler

### Code Execution
- `POST /api/code/execute` - Execute code in sandbox

---

**✅ All Backend Integrations Complete!**
