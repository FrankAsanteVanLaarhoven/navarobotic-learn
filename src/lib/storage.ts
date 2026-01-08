/**
 * File Storage Service
 * Supports AWS S3 and Cloudflare R2
 */

import { S3Client, PutObjectCommand, GetObjectCommand, DeleteObjectCommand } from '@aws-sdk/client-s3'
import { getSignedUrl } from '@aws-sdk/s3-request-presigner'

export interface StorageConfig {
  provider: 's3' | 'r2'
  accessKeyId: string
  secretAccessKey: string
  bucket: string
  region?: string
  endpoint?: string // Required for R2
}

export class StorageService {
  private client: S3Client
  private bucket: string
  private provider: 's3' | 'r2'

  constructor(config: StorageConfig) {
    this.bucket = config.bucket
    this.provider = config.provider

    const s3Config: any = {
      credentials: {
        accessKeyId: config.accessKeyId,
        secretAccessKey: config.secretAccessKey
      },
      region: config.region || 'us-east-1'
    }

    // Cloudflare R2 requires custom endpoint
    if (config.provider === 'r2' && config.endpoint) {
      s3Config.endpoint = config.endpoint
      s3Config.forcePathStyle = true
    }

    this.client = new S3Client(s3Config)
  }

  /**
   * Upload a file to storage
   */
  async uploadFile(
    key: string,
    body: Buffer | Uint8Array | string,
    contentType?: string,
    metadata?: Record<string, string>
  ): Promise<string> {
    const command = new PutObjectCommand({
      Bucket: this.bucket,
      Key: key,
      Body: body,
      ContentType: contentType,
      Metadata: metadata
    })

    await this.client.send(command)
    return this.getPublicUrl(key)
  }

  /**
   * Get a signed URL for file access (expires in 1 hour)
   */
  async getSignedUrl(key: string, expiresIn: number = 3600): Promise<string> {
    const command = new GetObjectCommand({
      Bucket: this.bucket,
      Key: key
    })

    return await getSignedUrl(this.client, command, { expiresIn })
  }

  /**
   * Delete a file from storage
   */
  async deleteFile(key: string): Promise<void> {
    const command = new DeleteObjectCommand({
      Bucket: this.bucket,
      Key: key
    })

    await this.client.send(command)
  }

  /**
   * Get public URL for a file
   */
  getPublicUrl(key: string): string {
    if (this.provider === 'r2') {
      // Cloudflare R2 public URL format
      const accountId = process.env.CLOUDFLARE_ACCOUNT_ID || ''
      return `https://${accountId}.r2.cloudflarestorage.com/${this.bucket}/${key}`
    } else {
      // AWS S3 public URL format
      const region = process.env.AWS_REGION || 'us-east-1'
      return `https://${this.bucket}.s3.${region}.amazonaws.com/${key}`
    }
  }
}

// Singleton instance
let storageService: StorageService | null = null

export function getStorageService(): StorageService | null {
  if (storageService) return storageService

  const provider = (process.env.STORAGE_PROVIDER || 's3') as 's3' | 'r2'
  const accessKeyId = process.env.STORAGE_ACCESS_KEY_ID || ''
  const secretAccessKey = process.env.STORAGE_SECRET_ACCESS_KEY || ''

  if (!accessKeyId || !secretAccessKey) {
    console.warn('Storage credentials not configured')
    return null
  }

  const config: StorageConfig = {
    provider,
    accessKeyId,
    secretAccessKey,
    bucket: process.env.STORAGE_BUCKET || '',
    region: process.env.STORAGE_REGION,
    endpoint: process.env.STORAGE_ENDPOINT
  }

  storageService = new StorageService(config)
  return storageService
}
