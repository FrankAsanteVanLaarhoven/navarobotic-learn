/**
 * Stripe Payment Integration
 * Handles subscriptions, one-time payments, and webhooks
 */

import Stripe from 'stripe'

if (!process.env.STRIPE_SECRET_KEY) {
  console.warn('Stripe secret key not configured')
}

export const stripe = new Stripe(process.env.STRIPE_SECRET_KEY || '', {
  apiVersion: '2024-12-18.acacia',
  typescript: true
})

export interface CreateCheckoutSessionParams {
  userId: string
  courseId?: string
  priceId: string
  successUrl: string
  cancelUrl: string
  metadata?: Record<string, string>
}

export interface CreateSubscriptionParams {
  userId: string
  priceId: string
  metadata?: Record<string, string>
}

/**
 * Create a Stripe Checkout Session for one-time payments
 */
export async function createCheckoutSession(
  params: CreateCheckoutSessionParams
): Promise<Stripe.Checkout.Session> {
  const session = await stripe.checkout.sessions.create({
    mode: 'payment',
    payment_method_types: ['card'],
    line_items: [
      {
        price: params.priceId,
        quantity: 1
      }
    ],
    customer_email: params.userId, // Replace with actual email lookup
    success_url: params.successUrl,
    cancel_url: params.cancelUrl,
    metadata: {
      userId: params.userId,
      courseId: params.courseId || '',
      ...params.metadata
    }
  })

  return session
}

/**
 * Create a Stripe Checkout Session for subscriptions
 */
export async function createSubscriptionSession(
  params: CreateSubscriptionParams
): Promise<Stripe.Checkout.Session> {
  const session = await stripe.checkout.sessions.create({
    mode: 'subscription',
    payment_method_types: ['card'],
    line_items: [
      {
        price: params.priceId,
        quantity: 1
      }
    ],
    customer_email: params.userId, // Replace with actual email lookup
    success_url: `${process.env.NEXT_PUBLIC_APP_URL}/student?session_id={CHECKOUT_SESSION_ID}`,
    cancel_url: `${process.env.NEXT_PUBLIC_APP_URL}/catalog`,
    metadata: {
      userId: params.userId,
      ...params.metadata
    }
  })

  return session
}

/**
 * Create or retrieve a Stripe customer
 */
export async function getOrCreateCustomer(
  userId: string,
  email: string,
  name?: string
): Promise<Stripe.Customer> {
  // Check if customer already exists (store customerId in database)
  // For now, create a new customer each time
  const customer = await stripe.customers.create({
    email,
    name,
    metadata: {
      userId
    }
  })

  return customer
}

/**
 * Handle Stripe webhook events
 */
export async function handleStripeWebhook(
  payload: string,
  signature: string
): Promise<Stripe.Event> {
  const webhookSecret = process.env.STRIPE_WEBHOOK_SECRET || ''

  if (!webhookSecret) {
    throw new Error('Stripe webhook secret not configured')
  }

  const event = stripe.webhooks.constructEvent(
    payload,
    signature,
    webhookSecret
  )

  return event
}

/**
 * Get subscription details
 */
export async function getSubscription(
  subscriptionId: string
): Promise<Stripe.Subscription> {
  return await stripe.subscriptions.retrieve(subscriptionId)
}

/**
 * Cancel a subscription
 */
export async function cancelSubscription(
  subscriptionId: string
): Promise<Stripe.Subscription> {
  return await stripe.subscriptions.cancel(subscriptionId)
}
