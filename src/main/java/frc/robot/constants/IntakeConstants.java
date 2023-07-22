package frc.robot.constants;

public final class IntakeConstants {
    public static final int wristMotorID = 11;

    public static final double pitchP = 0.0;
    public static final double pitchI = 0.0;
    public static final double pitchD = 0.0;

    public static final double pitchEncoderPositionConversionFactor = 0.0;
    public static final double pitchEncoderVelocityConversionFactor = 0.0;

    public static final double pitchEncoderOffset = 0.0;
    public static final boolean intakeMotorLInverted = true;
    public static final boolean intakeMotorRInverted = true;

    public static final int intakeMotorsCurrentLimit = 20;
    public static final int pitchMotorCurrentLimit = 20;

    private IntakeConstants() {
        // private constructor to prevent instantiation
    }
}
