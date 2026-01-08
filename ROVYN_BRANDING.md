# 🎨 Rovyn Brand Guidelines

## Brand Identity

**Rovyn — Learn robotics by building.**

Rovyn is a clean, builder-first robotics learning platform. The brand emphasizes hands-on learning, practical skills, and real-world application over hype.

## Logo

- **Wordmark**: "Rovyn" in a geometric sans-serif font
- **Ligature**: Subtle connection between "v" and "y" in the wordmark
- **Monogram**: Compact "R" mark for favicon/app icon (angled leg suggests motion/robot arm)
- **Usage**: Use wordmark for headers, monogram for small spaces (favicon, app icons)

## Color Palette

### Primary Colors
- **Primary (Electric Indigo)**: `#2E7DFF`
  - Use for: Primary CTAs, links, active states, brand elements
  - OKLCH: `oklch(0.58 0.18 260)`

- **Secondary (Teal)**: `#00C2A8`
  - Use for: Secondary actions, accents, highlights
  - OKLCH: `oklch(0.70 0.12 180)`

- **Accent (Amber)**: `#FFD166`
  - Use for: Warnings, highlights, special features
  - OKLCH: `oklch(0.85 0.12 85)`

### Neutral Colors
- **Ink (Near-Black)**: `#0B0F19`
  - Use for: Primary text, headings
  - OKLCH: `oklch(0.15 0.01 265)`

- **Surface (Off-White)**: `#F5F7FB`
  - Use for: Backgrounds, cards, surfaces
  - OKLCH: `oklch(0.97 0.002 265)`

## Typography

### Primary Font: Inter
- **Usage**: UI text, headings, body copy
- **Weights**: 400 (Regular), 500 (Medium), 600 (SemiBold), 700 (Bold)
- **Style**: Clean, modern, highly legible

### Code Font: JetBrains Mono
- **Usage**: Code blocks, terminal output, technical content
- **Weights**: 400 (Regular), 500 (Medium)
- **Style**: Monospace, developer-friendly

## Tone & Voice

### Principles
- **Builder-first**: Focus on practical skills and real outcomes
- **Concise**: Get to the point quickly
- **Demo > Hype**: Show, don't just tell
- **Professional but approachable**: Technical accuracy with friendly delivery

### Writing Style
- Use active voice
- Short sentences
- Clear, direct language
- Technical when needed, but always accessible
- Avoid marketing fluff

## Visual Style

### UI Elements
- **Rounded corners**: `rounded-xl` (0.75rem) for cards and buttons
- **White space**: Generous spacing for clarity
- **Shadows**: Minimal, subtle shadows
- **Icons**: Lucide icon set
- **Motion**: Subtle hover/slide animations

### Illustrations
- Simple robot arm motifs
- Grid overlays for technical feel
- No busy gradients
- Clean, minimal aesthetic

## Messaging

### Hero Copy
- **H1**: "Learn robotics by building."
- **Subhead**: "Courses, 3D simulation, and real-robot labs in one platform."

### Section Headlines
1. **Hands-on learning** — Short labs that ship to real robots.
2. **3D simulation** — Browser-based ROS/physics; fail fast, fix faster.
3. **AI video & feedback** — Auto-generated walkthroughs, instant checks.
4. **Career tracks** — From fundamentals to deployment.
5. **For campuses & teams** — Admin, analytics, SSO.

### CTAs
- Primary: "Start learning"
- Secondary: "Try the simulator"
- Tertiary: "Browse courses"

## Domain & URLs

### Primary Domain
- `rovyn.io` (canonical)
- Redirect `www.rovyn.io` → `rovyn.io`
- Redirect `http://` → `https://`

### Subdomains
- `app.rovyn.io` - Main application
- `docs.rovyn.io` - Documentation/curriculum
- `sim.rovyn.io` - Web simulator
- `api.rovyn.io` - API endpoints
- `status.rovyn.io` - System status
- `careers.rovyn.io` - Careers page

### URL Structure
- `/courses/ros-basics` - Course pages
- `/sim/...` - Simulation pages
- `/labs/...` - Lab pages
- `/docs/...` - Documentation

## Social Media

### Handles
- Primary: `@rovyn`
- Fallback: `@rovynrobotics` (if primary unavailable)

### Content Strategy
- Show real builds and projects
- Share student success stories
- Technical tutorials and tips
- Behind-the-scenes of platform development

## SEO & Meta Tags

### Default Meta Tags
```html
<title>Rovyn — Learn robotics by building</title>
<meta name="description" content="Courses, 3D simulation, and real-robot labs in one platform." />
<meta property="og:title" content="Rovyn" />
<meta property="og:description" content="Learn robotics by building." />
<meta property="og:url" content="https://rovyn.io" />
<meta property="og:type" content="website" />
<meta name="twitter:card" content="summary_large_image" />
```

## Legal Pages

Required pages:
- **Pricing** - Clear pricing tiers
- **For Campus** - Enterprise/education offerings
- **Docs** - Documentation hub
- **Privacy** - Privacy policy
- **Terms** - Terms of service
- **Status** - System status page
- **Data & Security** - Security practices, hosting region, backups, deletion policy

## Success Metrics

Track these in the first 60 days:
- **Activation**: % of visitors who run a sim or start the first lab
- **Learning outcome**: % completing "ROS Basics" within 7 days
- **Conversion**: free → paid or campus demo requests
- **Referral**: NPS on module completion

## Implementation Checklist

- [x] Update color palette in CSS
- [x] Update typography (Inter + JetBrains Mono)
- [x] Update landing page copy
- [x] Update meta tags
- [ ] Create logo assets (wordmark + monogram)
- [ ] Update favicon with "R" monogram
- [ ] Set up domain redirects
- [ ] Configure DNS records
- [ ] Set up email (SPF, DKIM, DMARC)
- [ ] Reserve social media handles
- [ ] Create legal pages
- [ ] Update all documentation
- [ ] Update email templates
- [ ] Set up analytics (Plausible or GA4)

---

**Brand Status**: ✅ Core branding implemented, assets pending
