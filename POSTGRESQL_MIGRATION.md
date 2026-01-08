# 🗄️ PostgreSQL Migration Guide

## Overview

This guide explains how to migrate from SQLite (development) to PostgreSQL (production).

## Prerequisites

- PostgreSQL 14+ installed and running
- Database credentials (host, port, database name, user, password)
- Prisma CLI installed

## Step 1: Update Prisma Schema

The schema is already compatible with PostgreSQL. Update the `datasource` in `prisma/schema.prisma`:

```prisma
datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}
```

## Step 2: Set Environment Variables

Add to your `.env` file:

```bash
# PostgreSQL Connection String
DATABASE_URL="postgresql://username:password@localhost:5432/navaroboticlearn?schema=public"
```

**Production format:**
```bash
DATABASE_URL="postgresql://user:password@host:5432/database?schema=public&sslmode=require"
```

## Step 3: Create Database

```bash
# Connect to PostgreSQL
psql -U postgres

# Create database
CREATE DATABASE navaroboticlearn;

# Create user (optional)
CREATE USER navauser WITH PASSWORD 'secure_password';
GRANT ALL PRIVILEGES ON DATABASE navaroboticlearn TO navauser;
```

## Step 4: Run Migrations

```bash
# Generate Prisma Client
bun run db:generate

# Push schema to PostgreSQL
bun run db:push

# Or create a migration
bun run db:migrate dev --name init
```

## Step 5: Migrate Data (if needed)

If you have existing SQLite data:

```bash
# Export from SQLite
sqlite3 db/custom.db .dump > backup.sql

# Convert SQLite dump to PostgreSQL format
# Use a tool like pgloader or manual conversion
pgloader sqlite://db/custom.db postgresql://user:pass@localhost/navaroboticlearn
```

## Step 6: Verify Migration

```bash
# Test connection
bun -e "import { PrismaClient } from '@prisma/client'; const p = new PrismaClient(); p.\$connect().then(() => console.log('Connected!')).catch(e => console.error(e))"

# Check tables
psql -U postgres -d navaroboticlearn -c "\dt"
```

## Production Considerations

### Connection Pooling

For production, use a connection pooler like PgBouncer:

```bash
DATABASE_URL="postgresql://user:pass@pgbouncer:6432/db?pgbouncer=true"
```

### SSL/TLS

Always use SSL in production:

```bash
DATABASE_URL="postgresql://user:pass@host:5432/db?sslmode=require"
```

### Backup Strategy

```bash
# Daily backups
pg_dump -U user -d navaroboticlearn > backup_$(date +%Y%m%d).sql

# Restore
psql -U user -d navaroboticlearn < backup_20240101.sql
```

## Troubleshooting

### Connection Issues

- Check PostgreSQL is running: `pg_isready`
- Verify credentials in `.env`
- Check firewall rules
- Verify SSL settings

### Migration Errors

- Clear Prisma cache: `rm -rf node_modules/.prisma`
- Regenerate client: `bun run db:generate`
- Check schema syntax

### Performance

- Add indexes for frequently queried fields
- Use connection pooling
- Monitor query performance with `EXPLAIN ANALYZE`

## Rollback Plan

If you need to rollback to SQLite:

1. Export PostgreSQL data
2. Update `prisma/schema.prisma` to use SQLite
3. Run `bun run db:push`
4. Import data

---

**✅ Migration Complete!** Your database is now running on PostgreSQL.
