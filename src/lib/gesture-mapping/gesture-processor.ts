/**
 * Gesture-to-Action Mapping System
 * Maps recognized hand gestures to robot commands
 * Supports custom gesture mapping and calibration
 */

export interface Gesture {
  id: string
  name: string
  description: string
  trigger: {
    type: 'pose' | 'gesture' | 'motion'
    requirements: {
      fingers?: {
        thumb?: 'extended' | 'curled' | 'unknown'
        index?: 'extended' | 'curled' | 'unknown'
        middle?: 'extended' | 'curled' | 'unknown'
        ring?: 'extended' | 'curled' | 'unknown'
        pinky?: 'extended' | 'curled' | 'unknown'
      }
      palm?: {
        orientation?: 'up' | 'down' | 'left' | 'right' | 'unknown'
      }
      wrist?: {
        orientation?: 'up' | 'down' | 'left' | 'right' | 'neutral' | 'unknown'
      }
    }
  }
  action: {
    type: 'movement' | 'manipulation' | 'interaction' | 'control'
    target: string
    parameters: Record<string, any>
  }
  confidence: number
}

export interface GestureAction {
  id: string
  gestureId: string
  actionType: 'move' | 'rotate' | 'gripper' | 'pose' | 'reset' | 'custom'
  joint: string
  value: number
}

export class GestureProcessor {
  private gestureRecognizer: any = null
  private currentGestures: Gesture[] = []
  private gestureMappings: Map<string, GestureAction[]> = new Map()
  private isCalibrated = false

  /**
   * Initialize MediaPipe Hands
   */
  async initialize(): Promise<void> {
    try {
      // In production, would use MediaPipe
      // For now, we'll use a simplified gesture recognition
      this.isCalibrated = true
      console.log('Gesture Processor initialized')
    } catch (error) {
      console.error('Failed to initialize Gesture Processor:', error)
      throw error
    }
  }

  /**
   * Detect gestures from hand landmarks
   */
  async detectGestures(landmarks: number[][]): Promise<Gesture[]> {
    if (!this.isCalibrated) {
      await this.initialize()
    }

    const gestures: Gesture[] = []
    const fingerStates = this.calculateFingerStates(landmarks)
    const palmOrientation = this.calculatePalmOrientation(landmarks)

    // Built-in gestures
    const builtinGestures: Gesture[] = [
      {
        id: 'thumbs-up',
        name: 'Thumbs Up',
        description: 'Thumb extended, other fingers curled',
        trigger: {
          type: 'pose',
          requirements: {
            fingers: {
              thumb: 'extended',
              index: 'curled',
              middle: 'curled',
              ring: 'curled',
              pinky: 'curled'
            },
            palm: { orientation: 'up' }
          }
        },
        action: {
          type: 'movement',
          target: 'base',
          parameters: { action: 'arm_up' }
        },
        confidence: 0.85
      },
      {
        id: 'point',
        name: 'Point',
        description: 'Index finger extended, pointing',
        trigger: {
          type: 'gesture',
          requirements: {
            fingers: {
              index: 'extended',
              middle: 'curled',
              ring: 'curled',
              pinky: 'curled'
            }
          }
        },
        action: {
          type: 'movement',
          target: 'arm',
          parameters: { action: 'extend' }
        },
        confidence: 0.9
      },
      {
        id: 'pinch',
        name: 'Pinch',
        description: 'Thumb and index finger touching',
        trigger: {
          type: 'gesture',
          requirements: {
            fingers: {
              thumb: 'extended',
              index: 'extended'
            }
          }
        },
        action: {
          type: 'manipulation',
          target: 'gripper',
          parameters: { action: 'close' }
        },
        confidence: 0.95
      },
      {
        id: 'open-palm',
        name: 'Open Palm',
        description: 'All fingers extended',
        trigger: {
          type: 'gesture',
          requirements: {
            fingers: {
              thumb: 'extended',
              index: 'extended',
              middle: 'extended',
              ring: 'extended',
              pinky: 'extended'
            },
            palm: { orientation: 'up' }
          }
        },
        action: {
          type: 'manipulation',
          target: 'gripper',
          parameters: { action: 'open' }
        },
        confidence: 0.9
      }
    ]

    // Check if gestures match
    builtinGestures.forEach(gesture => {
      if (this.matchesGesture(gesture, fingerStates, palmOrientation)) {
        gestures.push(gesture)
      }
    })

    return gestures
  }

  private calculateFingerStates(landmarks: number[][]): Record<string, 'extended' | 'curled' | 'unknown'> {
    const states: Record<string, 'extended' | 'curled' | 'unknown'> = {
      thumb: 'unknown',
      index: 'unknown',
      middle: 'unknown',
      ring: 'unknown',
      pinky: 'unknown'
    }

    // Simplified finger state detection
    if (landmarks.length >= 21) {
      // Thumb
      states.thumb = landmarks[4] && landmarks[4][1] < landmarks[2][1] ? 'extended' : 'curled'
      // Index
      states.index = landmarks[8] && landmarks[8][1] < landmarks[5][1] ? 'extended' : 'curled'
      // Middle
      states.middle = landmarks[12] && landmarks[12][1] < landmarks[9][1] ? 'extended' : 'curled'
      // Ring
      states.ring = landmarks[16] && landmarks[16][1] < landmarks[13][1] ? 'extended' : 'curled'
      // Pinky
      states.pinky = landmarks[20] && landmarks[20][1] < landmarks[17][1] ? 'extended' : 'curled'
    }

    return states
  }

  private calculatePalmOrientation(landmarks: number[][]): 'up' | 'down' | 'left' | 'right' | 'unknown' {
    if (landmarks.length < 5) return 'unknown'
    
    const wrist = landmarks[0]
    const middleFingerMCP = landmarks[9]
    
    if (wrist[1] < middleFingerMCP[1]) {
      return 'up'
    } else if (wrist[1] > middleFingerMCP[1]) {
      return 'down'
    }
    
    return 'unknown'
  }

  private matchesGesture(gesture: Gesture, fingerStates: Record<string, string>, palmOrientation: string): boolean {
    const req = gesture.trigger.requirements
    
    if (req.fingers) {
      for (const [finger, state] of Object.entries(req.fingers)) {
        if (fingerStates[finger] !== state && state !== 'unknown') {
          return false
        }
      }
    }
    
    if (req.palm?.orientation && req.palm.orientation !== palmOrientation && req.palm.orientation !== 'unknown') {
      return false
    }
    
    return true
  }

  /**
   * Map gesture to action
   */
  mapGestureToAction(gesture: Gesture): GestureAction[] {
    const actions: GestureAction[] = []

    switch (gesture.id) {
      case 'thumbs-up':
        actions.push({
          id: `${gesture.id}-arm-up`,
          gestureId: gesture.id,
          actionType: 'move',
          joint: 'shoulder',
          value: -1.0
        })
        break

      case 'point':
        actions.push({
          id: `${gesture.id}-arm-extend`,
          gestureId: gesture.id,
          actionType: 'rotate',
          joint: 'elbow',
          value: 0.5
        })
        break

      case 'pinch':
        actions.push({
          id: `${gesture.id}-gripper-close`,
          gestureId: gesture.id,
          actionType: 'gripper',
          joint: 'gripper',
          value: 0.0
        })
        break

      case 'open-palm':
        actions.push({
          id: `${gesture.id}-gripper-open`,
          gestureId: gesture.id,
          actionType: 'gripper',
          joint: 'gripper',
          value: 1.0
        })
        break
    }

    return actions
  }
}

/**
 * Factory function to create gesture processor
 */
export function createGestureProcessor(): GestureProcessor {
  return new GestureProcessor()
}
