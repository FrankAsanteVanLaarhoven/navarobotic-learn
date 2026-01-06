export default function Loading() {
  return (
    <div className="min-h-screen flex items-center justify-center bg-background">
      <div className="flex flex-col items-center gap-4">
        {/* Loading Spinner */}
        <div className="relative w-12 h-12">
          <div className="absolute inset-0 flex items-center justify-center">
            <div className="w-8 h-8 border-4 border-primary border-t-transparent rounded-full animate-spin" />
          </div>
        </div>

        {/* Loading Text */}
        <div className="space-y-2 text-center">
          <p className="text-lg font-semibold">Loading...</p>
          <p className="text-sm text-muted-foreground">Please wait</p>
        </div>
      </div>

      {/* Loading Progress Bar */}
      <div className="w-64">
        <div className="h-2 bg-primary/20 rounded-full overflow-hidden">
          <div className="h-full bg-primary animate-pulse rounded-full" style={{ width: '60%' }} />
        </div>
      </div>

      {/* Loading Skeleton for Cards */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mt-12 max-w-6xl">
        {[1, 2, 3].map((i) => (
          <div key={i} className="space-y-3">
            <div className="h-6 bg-muted/20 rounded-lg animate-pulse" />
            <div className="h-4 bg-muted/20 rounded-lg animate-pulse" style={{ animationDelay: `${i * 0.2}s` }} />
            <div className="h-4 bg-muted/20 rounded-lg animate-pulse" style={{ animationDelay: `${i * 0.3}s` }} />
          </div>
        ))}
      </div>
    </div>
  )
}
