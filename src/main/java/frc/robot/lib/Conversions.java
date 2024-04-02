package frc.robot.lib;

public class Conversions {
    public static double toMeters(double feet) // Feet to meters
    {
        return feet * 0.3048;
    }

    public static double toFeet(double meters) // Meters to feet
    {
        return meters * 3.2808399;
    }
}
