import java.util.HashMap;
import java.util.Map;

public class ProjectileTrajectoryLookup {
    
    // Lookup table structure: Key = encoded params, Value = trajectory data
    private Map<String, TrajectoryData> lookupTable;
    
    // Physical constants
    private static final double GRAVITY = 9.81; // m/s^2
    
    public ProjectileTrajectoryLookup() {
        lookupTable = new HashMap<>();
        buildLookupTable();
    }
    
    // Encode projectile parameters into a lookup key
    private String encodeKey(double velocity, double angle, double height) {
        // Round values to reduce table size (adjust precision as needed)
        int v = (int) Math.round(velocity);
        int a = (int) Math.round(angle);
        int h = (int) Math.round(height * 10); // 0.1m precision
        return String.format("%d_%d_%d", v, a, h);
    }
    
    // Decode key back to parameters
    public ProjectileParams decodeKey(String key) {
        String[] parts = key.split("_");
        if (parts.length != 3) {
            throw new IllegalArgumentException("Invalid key format");
        }
        
        double velocity = Double.parseDouble(parts[0]);
        double angle = Double.parseDouble(parts[1]);
        double height = Double.parseDouble(parts[2]) / 10.0;
        
        return new ProjectileParams(velocity, angle, height);
    }
    
    // Build the lookup table with pre-calculated trajectories
    private void buildLookupTable() {
        // Example: Build table for common projectile parameters
        for (int velocity = 10; velocity <= 50; velocity += 5) {
            for (int angle = 15; angle <= 75; angle += 5) {
                for (int height = 0; height <= 20; height += 5) {
                    String key = encodeKey(velocity, angle, height);
                    TrajectoryData data = calculateTrajectory(velocity, angle, height);
                    lookupTable.put(key, data);
                }
            }
        }
        System.out.println("Lookup table built with " + lookupTable.size() + " entries");
    }
    
    // Calculate trajectory physics
    private TrajectoryData calculateTrajectory(double v0, double angleDeg, double h0) {
        double angleRad = Math.toRadians(angleDeg);
        double vx = v0 * Math.cos(angleRad);
        double vy = v0 * Math.sin(angleRad);
        
        // Time of flight: solve -0.5*g*t^2 + vy*t + h0 = 0
        double discriminant = vy * vy + 2 * GRAVITY * h0;
        double timeOfFlight = (vy + Math.sqrt(discriminant)) / GRAVITY;
        
        // Maximum height
        double maxHeight = h0 + (vy * vy) / (2 * GRAVITY);
        
        // Range
        double range = vx * timeOfFlight;
        
        // Time to max height
        double timeToMaxHeight = vy / GRAVITY;
        
        return new TrajectoryData(range, maxHeight, timeOfFlight, timeToMaxHeight);
    }
    
    // Lookup trajectory data
    public TrajectoryData lookup(double velocity, double angle, double height) {
        String key = encodeKey(velocity, angle, height);
        TrajectoryData data = lookupTable.get(key);
        
        if (data == null) {
            // Fallback: calculate on-the-fly if not in table
            System.out.println("Key not found, calculating: " + key);
            return calculateTrajectory(velocity, angle, height);
        }
        
        return data;
    }
    
    // Get position at time t
    public Position getPositionAt(double velocity, double angle, double height, double t) {
        double angleRad = Math.toRadians(angle);
        double vx = velocity * Math.cos(angleRad);
        double vy = velocity * Math.sin(angleRad);
        
        double x = vx * t;
        double y = height + vy * t - 0.5 * GRAVITY * t * t;
        
        return new Position(x, y);
    }
    
    // Data classes
    public static class TrajectoryData {
        public final double range;
        public final double maxHeight;
        public final double timeOfFlight;
        public final double timeToMaxHeight;
        
        public TrajectoryData(double range, double maxHeight, double timeOfFlight, double timeToMaxHeight) {
            this.range = range;
            this.maxHeight = maxHeight;
            this.timeOfFlight = timeOfFlight;
            this.timeToMaxHeight = timeToMaxHeight;
        }
        
        @Override
        public String toString() {
            return String.format("Range: %.2fm, Max Height: %.2fm, Time: %.2fs", 
                range, maxHeight, timeOfFlight);
        }
    }
    
    public static class ProjectileParams {
        public final double velocity;
        public final double angle;
        public final double height;
        
        public ProjectileParams(double velocity, double angle, double height) {
            this.velocity = velocity;
            this.angle = angle;
            this.height = height;
        }
    }
    
    public static class Position {
        public final double x;
        public final double y;
        
        public Position(double x, double y) {
            this.x = x;
            this.y = y;
        }
        
        @Override
        public String toString() {
            return String.format("(%.2f, %.2f)", x, y);
        }
    }
    
    // Example usage
    public static void main(String[] args) {
        ProjectileTrajectoryLookup lookup = new ProjectileTrajectoryLookup();
        
        // Lookup trajectory
        TrajectoryData data = lookup.lookup(30, 45, 0);
        System.out.println("Trajectory for v=30m/s, angle=45°, h=0m:");
        System.out.println(data);
        
        // Decode a key
        String key = "30_45_0";
        ProjectileParams params = lookup.decodeKey(key);
        System.out.println("\nDecoded key '" + key + "':");
        System.out.println("Velocity: " + params.velocity + " m/s");
        System.out.println("Angle: " + params.angle + "°");
        System.out.println("Height: " + params.height + " m");
        
        // Get position at specific time
        Position pos = lookup.getPositionAt(30, 45, 0, 2.0);
        System.out.println("\nPosition at t=2s: " + pos);
    }
}
