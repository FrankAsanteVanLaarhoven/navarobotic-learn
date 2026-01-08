# 🚀 SOTA Features Setup Guide

Complete setup instructions for all integrated features.

## 📋 Quick Setup Checklist

- [ ] Set GEMINI_API_KEY in .env
- [ ] Configure robot connection (optional)
- [ ] Test haptics page
- [ ] Setup ROSBridge (optional, for WebROS IDE)

---

## 1. LLM Tutor Setup

### Step 1: Get Gemini API Key

1. Visit https://aistudio.google.com/app/apikey
2. Sign in with your Google account
3. Click "Create API Key"
4. Copy the API key

### Step 2: Add to .env File

Add this line to your `.env` file:

```env
GEMINI_API_KEY=your_actual_api_key_here
```

**Note**: Replace `your_actual_api_key_here` with your actual API key from Google AI Studio.

### Step 3: Verify

Restart your dev server and test the LLM Tutor component. It should now use the real Gemini API instead of mock responses.

---

## 2. Haptics Test Page

### Access the Test Page

1. Start your dev server:
   ```bash
   npm run dev
   # or
   bun run dev
   ```

2. Navigate to: http://localhost:3000/haptics-test

### Connect a Gamepad

1. **Xbox Controller**: Connect via USB or Bluetooth
2. **PlayStation Controller**: Connect via USB or Bluetooth
3. **Generic Gamepad**: Any gamepad compatible with the Gamepad API

The page will automatically detect when a gamepad is connected.

### Test Haptic Feedback

- Click the different haptic event buttons
- Adjust intensity and duration sliders
- Try collision feedback (light, medium, heavy)
- Test grip resistance and success pulse

**Note**: If no gamepad is available, audio-based spectral haptics will still work.

---

## 3. Robot Connection Configuration

### Option A: Use Environment Variables (Recommended)

Add these to your `.env` file:

```env
# Robot Connection Settings
NEXT_PUBLIC_ROBOT_ID=unitree-g1-1
NEXT_PUBLIC_ROBOT_TYPE=unitree-g1
NEXT_PUBLIC_ROBOT_CONNECTION_TYPE=websocket
NEXT_PUBLIC_ROBOT_HOST=localhost
NEXT_PUBLIC_ROBOT_PORT=8080
```

### Option B: Modify Component Directly

Edit `/src/components/sim2real/Sim2RealManager.tsx` and update the connection config:

```typescript
const connection = createRobotConnection({
  robotId: 'your-robot-id',
  type: 'unitree-g1',
  connectionType: 'websocket', // or 'mqtt', 'rest', 'ros2'
  host: 'your-robot-host',
  port: 8080
})
```

### Connection Types

- **websocket**: Browser-based WebSocket connection
- **mqtt**: MQTT broker connection
- **rest**: REST API connection
- **ros2**: ROS2 native connection

---

## 4. ROSBridge Setup (for WebROS IDE)

### Prerequisites

- ROS or ROS2 installed
- ROSBridge suite installed

### Quick Setup

#### For ROS2 (Recommended)

1. **Install ROSBridge**:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install ros-humble-rosbridge-suite
   
   # macOS
   brew install ros-humble-rosbridge-suite
   ```

2. **Start ROSBridge**:
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Start ROSBridge
   ros2 launch rosbridge_server rosbridge_websocket.launch.py
   ```

3. **Or use the setup script**:
   ```bash
   chmod +x scripts/setup-rosbridge.sh
   ./scripts/setup-rosbridge.sh
   ```

#### For ROS1

1. **Install ROSBridge**:
   ```bash
   sudo apt-get install ros-melodic-rosbridge-suite
   ```

2. **Start ROSBridge**:
   ```bash
   source /opt/ros/melodic/setup.bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

### Configure WebSocket URL

By default, WebROS IDE connects to `ws://localhost:9090`. To change this:

1. **Via Environment Variable**:
   ```env
   NEXT_PUBLIC_ROS_BRIDGE_URL=ws://your-host:9090
   ```

2. **Or in code**:
   ```typescript
   const ide = new WebROSIDE()
   await ide.initialize('ws://your-host:9090')
   ```

### Verify Connection

1. Start ROSBridge
2. Initialize WebROS IDE in your application
3. Check the terminal output - you should see "Connected to ROS Bridge"

---

## 5. Complete .env Template

Here's a complete `.env` template with all SOTA features:

```env
# ============================================
# SOTA Features Configuration
# ============================================

# LLM Tutor - Gemini API
GEMINI_API_KEY=your_gemini_api_key_here
GOOGLE_AI_API_KEY=your_gemini_api_key_here  # Alternative name

# WebROS IDE - ROSBridge
NEXT_PUBLIC_ROS_BRIDGE_URL=ws://localhost:9090

# Sim2Real Manager - Robot Connection
NEXT_PUBLIC_ROBOT_ID=unitree-g1-1
NEXT_PUBLIC_ROBOT_TYPE=unitree-g1
NEXT_PUBLIC_ROBOT_CONNECTION_TYPE=websocket
NEXT_PUBLIC_ROBOT_HOST=localhost
NEXT_PUBLIC_ROBOT_PORT=8080

# ============================================
# Other Platform Settings (if needed)
# ============================================

# Database
DATABASE_URL="file:./db/custom.db"

# NextAuth
NEXTAUTH_SECRET=your-secret-key-here
NEXTAUTH_URL=http://localhost:3000

# Storage (optional)
STORAGE_PROVIDER=s3
STORAGE_ACCESS_KEY_ID=your-access-key
STORAGE_SECRET_ACCESS_KEY=your-secret-key
STORAGE_BUCKET=your-bucket-name
STORAGE_REGION=us-east-1
```

---

## 6. Testing All Features

### Test LLM Tutor

```bash
# 1. Ensure GEMINI_API_KEY is set in .env
# 2. Start dev server
npm run dev

# 3. Navigate to a page with LLMTutor component
# 4. Ask: "Explain inverse kinematics"
```

### Test Haptics

```bash
# 1. Start dev server
npm run dev

# 2. Navigate to http://localhost:3000/haptics-test
# 3. Connect a gamepad
# 4. Test different haptic patterns
```

### Test WebROS IDE

```bash
# 1. Start ROSBridge
./scripts/setup-rosbridge.sh

# 2. In another terminal, start dev server
npm run dev

# 3. Initialize WebROS IDE in your app
# 4. Check terminal for "Connected to ROS Bridge"
```

### Test Sim2Real Manager

```bash
# 1. Ensure robot connection settings are configured
# 2. Start dev server
npm run dev

# 3. Navigate to page with Sim2RealManager
# 4. Switch between Simulation and Reality domains
```

### Test Digital Twin

```bash
# 1. Use MultiRobotScene with enablePhysics={true}
# 2. Watch robot positions sync in real-time
# 3. Check browser console for twin updates
```

---

## 7. Troubleshooting

### LLM Tutor Issues

**Problem**: "API key not configured"
- **Solution**: Check `.env` file has `GEMINI_API_KEY` set
- **Solution**: Restart dev server after adding env var
- **Solution**: Verify API key is valid at https://aistudio.google.com/app/apikey

**Problem**: "Failed to generate response"
- **Solution**: Check API quota/limits
- **Solution**: Verify internet connection
- **Solution**: Check browser console for detailed errors

### Haptics Issues

**Problem**: "Haptics not supported"
- **Solution**: Connect a compatible gamepad
- **Solution**: Check browser supports Gamepad API (Chrome, Firefox, Edge)
- **Solution**: Audio-based haptics should still work

**Problem**: Gamepad not detected
- **Solution**: Connect gamepad before opening page
- **Solution**: Refresh page after connecting
- **Solution**: Check gamepad is compatible with Gamepad API

### WebROS IDE Issues

**Problem**: "Failed to connect to ROS Bridge"
- **Solution**: Ensure ROSBridge is running: `ros2 launch rosbridge_server rosbridge_websocket.launch.py`
- **Solution**: Check WebSocket URL matches ROSBridge port
- **Solution**: Verify firewall allows WebSocket connections
- **Solution**: Check ROSBridge logs for errors

**Problem**: "ROS message parsing failed"
- **Solution**: Verify ROSBridge is sending valid JSON
- **Solution**: Check ROS topics are properly configured
- **Solution**: Review browser console for specific errors

### Sim2Real Manager Issues

**Problem**: "Physical robot not connected"
- **Solution**: Verify robot is powered on and accessible
- **Solution**: Check connection settings (host, port, robotId)
- **Solution**: Verify connection type matches robot setup
- **Solution**: Review network connectivity

**Problem**: "Telemetry not updating"
- **Solution**: Check robot is sending telemetry data
- **Solution**: Verify subscription to telemetry events
- **Solution**: Check browser console for connection errors

---

## 8. Production Deployment

### Environment Variables

For production, set environment variables in your hosting platform:

- **Vercel**: Project Settings → Environment Variables
- **Netlify**: Site Settings → Environment Variables
- **Docker**: Use `-e` flags or `.env` file
- **Kubernetes**: Use ConfigMaps or Secrets

### Important Notes

1. **NEXT_PUBLIC_*** variables are exposed to the browser
2. Keep sensitive keys (like `GEMINI_API_KEY`) server-side only
3. Use different API keys for development and production
4. Set up rate limiting for production APIs

---

## 9. Next Steps

1. ✅ Set up GEMINI_API_KEY
2. ✅ Test haptics page
3. ✅ Configure robot connection (if needed)
4. ✅ Setup ROSBridge (if using WebROS IDE)
5. ✅ Test all features
6. ✅ Deploy to production

---

## Support

For issues:
1. Check browser console for errors
2. Review terminal logs
3. Verify all environment variables are set
4. Check external services (ROS, robot) are running

**All features are ready to use! 🚀**
