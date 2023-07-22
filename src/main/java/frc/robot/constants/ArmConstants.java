package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/

    public static final int elevatorMotorId = 9;
    public static final int elbowMotorId = 10;
    public static final int wristMotorId = 11;
    public static final int intakeMotorOneId = 12;
    public static final int intakeMotorTwoId = 13;

    public static final int topBreakerId = 0;
    public static final int bottomBreakerId = 1;

    public static final int topLinebreakerId = 0;
    public static final int bottomLinebreakerId = 1;

    public static final boolean elevatorMotorInverted = false;
    public static final boolean elbowMotorInverted = true;
    public static final boolean intakeMotorInverted = false;

    //Need very small tune
    public static final double elevatorP = 100.0; //TODO: Needs tuning
    public static final double elevatorI = 0.0;
    public static final double elevatorD = 0.0;

    //val elevatorTrapezoidConstraints = TrapezoidProfile.Constraints(50.0, 30.0)
    //tune:
    public static final double elbowP = 4.0; //TODO: Needs tuning
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;

    //maybe change:
    public static final ArmFeedforward elbowFF = new ArmFeedforward(0.13, 0.50, 1.00); //might need to change kg

    //tune:
    public static final double wristP = 5.0;
    public static final double wristI = 0.0;
    public static final double wristD = 0.0;

    //maybe change:
    public static final ArmFeedforward wristFF = new ArmFeedforward(0.13, 0.20, 1.00);

    public static final double elevatorMinHeight = 0.05;
    public static final double elevatorMaxHeight = 0.948;

    public static final double elbowMaxRotation = Math.toRadians(130.0);
    public static final double elbowMinRotation = -(1.0 / 2.0) * Math.PI;

    public static final double wristMaxRotation = (Math.PI / 2.0);
    public static final double wristMinRotation = -(Math.PI / 2.0);

    // multipliers for unit conversion and stuff
    // these values obtained from tuning
    public static final double elevatorEncoderVelocityConversionFactor =
        (0.003010870139 * 2.4) / 60.0; //this should turn revs/min to meters/sec
    public static final double elevatorEncoderPositionConversionFactor = 0.003010870139 * 2.4; //this should turn revs to meters

    //could need small tuning:
    public static final double elbowEncoderPosOffset = 2.088;
    public static final double wristEncoderPosOffset = 2.767;

    public static final double elevatorZeroingVoltage = -1.0;

    private ArmConstants() {
        // private constructor to prevent instantiation
    }
}
