import { NextResponse } from 'next/server'
import { handleStripeWebhook } from '@/lib/stripe'
import { db } from '@/lib/db'
import { getEmailService } from '@/lib/email'

export async function POST(request: Request) {
  const body = await request.text()
  const signature = request.headers.get('stripe-signature')

  if (!signature) {
    return NextResponse.json(
      { error: 'Missing signature' },
      { status: 400 }
    )
  }

  try {
    const event = await handleStripeWebhook(body, signature)

    // Handle different event types
    switch (event.type) {
      case 'checkout.session.completed':
        const session = event.data.object as any
        const userId = session.metadata?.userId
        const courseId = session.metadata?.courseId

        if (userId && courseId) {
          // Create enrollment
          await db.enrollment.create({
            data: {
              userId,
              courseId,
              progress: 0
            }
          })

          // Send enrollment email
          const user = await db.user.findUnique({ where: { id: userId } })
          const course = await db.course.findUnique({ where: { id: courseId } })
          const emailService = getEmailService()

          if (user && course && emailService) {
            await emailService.sendEnrollmentEmail(
              user.email,
              course.title,
              `${process.env.NEXT_PUBLIC_APP_URL}/courses/${course.slug}`
            )
          }
        }
        break

      case 'customer.subscription.created':
      case 'customer.subscription.updated':
        // Handle subscription updates
        break

      case 'customer.subscription.deleted':
        // Handle subscription cancellation
        break

      default:
        console.log(`Unhandled event type: ${event.type}`)
    }

    return NextResponse.json({ received: true })
  } catch (error: any) {
    console.error('Stripe webhook error:', error)
    return NextResponse.json(
      { error: error.message },
      { status: 400 }
    )
  }
}
