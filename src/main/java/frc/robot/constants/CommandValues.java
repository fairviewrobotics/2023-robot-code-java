package frc.robot.constants;

public class CommandValues {
    // Swerve
    public static boolean fieldOriented = true;

    // Starting Config: Cube, with Middle Place, and Ground Pickup
    public static boolean cube = true; // Both
    public static boolean middlePlace = true; // Place
    public static boolean floor = false; // Place
    public static boolean chute = false; // Pickup
    public static boolean pickup = false;
    public static boolean shelf = false;

    // Vision
    public static boolean vision = false;

    // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
    public static boolean ground = true; // Pickup
    public static boolean cone = false; // Both
    public static boolean highPlace = false; // Place

    public static boolean balancing = false;
    public static boolean balanced = false;
    public static boolean auto = false;
    public static boolean visionIsMovingRobot = false;
}
