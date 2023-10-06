package team.ironriders.lib;

public class Utils {
    public static boolean isWithinTolerance(double input, double target, double tolerance) {
        double difference = Math.abs(input - target);
        return difference <= tolerance;
    }
}
