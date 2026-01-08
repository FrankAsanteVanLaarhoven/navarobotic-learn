/**
 * Email Service
 * Supports SendGrid and Resend
 */

export interface EmailConfig {
  provider: 'sendgrid' | 'resend'
  apiKey: string
  fromEmail: string
  fromName?: string
}

export interface EmailOptions {
  to: string | string[]
  subject: string
  html?: string
  text?: string
  templateId?: string // For SendGrid templates
  templateData?: Record<string, any>
}

export class EmailService {
  private provider: 'sendgrid' | 'resend'
  private apiKey: string
  private fromEmail: string
  private fromName: string

  constructor(config: EmailConfig) {
    this.provider = config.provider
    this.apiKey = config.apiKey
    this.fromEmail = config.fromEmail
    this.fromName = config.fromName || 'Rovyn'
  }

  /**
   * Send an email
   */
  async sendEmail(options: EmailOptions): Promise<void> {
    if (this.provider === 'sendgrid') {
      await this.sendWithSendGrid(options)
    } else {
      await this.sendWithResend(options)
    }
  }

  /**
   * Send email using SendGrid
   */
  private async sendWithSendGrid(options: EmailOptions): Promise<void> {
    const sgMail = await import('@sendgrid/mail')
    sgMail.default.setApiKey(this.apiKey)

    const msg: any = {
      to: options.to,
      from: {
        email: this.fromEmail,
        name: this.fromName
      },
      subject: options.subject
    }

    if (options.templateId) {
      msg.templateId = options.templateId
      msg.dynamicTemplateData = options.templateData
    } else {
      msg.html = options.html
      msg.text = options.text
    }

    await sgMail.default.send(msg)
  }

  /**
   * Send email using Resend
   */
  private async sendWithResend(options: EmailOptions): Promise<void> {
    const resend = await import('resend')
    const resendClient = new resend.Resend(this.apiKey)

    await resendClient.emails.send({
      from: `${this.fromName} <${this.fromEmail}>`,
      to: Array.isArray(options.to) ? options.to : [options.to],
      subject: options.subject,
      html: options.html,
      text: options.text
    })
  }

  /**
   * Send welcome email
   */
  async sendWelcomeEmail(to: string, name: string): Promise<void> {
    await this.sendEmail({
      to,
      subject: 'Welcome to Rovyn!',
      html: `
        <h1>Welcome, ${name}!</h1>
        <p>Thank you for joining Rovyn. Learn robotics by building—start your first lab today!</p>
        <a href="${process.env.NEXT_PUBLIC_APP_URL}/catalog">Browse Courses</a>
      `
    })
  }

  /**
   * Send course enrollment email
   */
  async sendEnrollmentEmail(
    to: string,
    courseName: string,
    courseUrl: string
  ): Promise<void> {
    await this.sendEmail({
      to,
      subject: `You've enrolled in ${courseName}`,
      html: `
        <h1>Course Enrollment Confirmed</h1>
        <p>You've successfully enrolled in <strong>${courseName}</strong>.</p>
        <a href="${courseUrl}">Start Learning</a>
      `
    })
  }

  /**
   * Send password reset email
   */
  async sendPasswordResetEmail(to: string, resetToken: string): Promise<void> {
    const resetUrl = `${process.env.NEXT_PUBLIC_APP_URL}/auth/reset-password?token=${resetToken}`
    await this.sendEmail({
      to,
      subject: 'Reset Your Password',
      html: `
        <h1>Password Reset Request</h1>
        <p>Click the link below to reset your password:</p>
        <a href="${resetUrl}">Reset Password</a>
        <p>This link will expire in 1 hour.</p>
      `
    })
  }

  /**
   * Send course completion email
   */
  async sendCourseCompletionEmail(
    to: string,
    courseName: string,
    certificateUrl: string
  ): Promise<void> {
    await this.sendEmail({
      to,
      subject: `Congratulations! You completed ${courseName}`,
      html: `
        <h1>Course Completed! 🎉</h1>
        <p>Congratulations on completing <strong>${courseName}</strong>!</p>
        <a href="${certificateUrl}">View Certificate</a>
      `
    })
  }
}

// Singleton instance
let emailService: EmailService | null = null

export function getEmailService(): EmailService | null {
  if (emailService) return emailService

  const provider = (process.env.EMAIL_PROVIDER || 'resend') as 'sendgrid' | 'resend'
  const apiKey = process.env.EMAIL_API_KEY || ''
  const fromEmail = process.env.EMAIL_FROM || 'noreply@rovyn.io'

  if (!apiKey) {
    console.warn('Email service not configured')
    return null
  }

  emailService = new EmailService({
    provider,
    apiKey,
    fromEmail,
    fromName: 'Rovyn'
  })

  return emailService
}
