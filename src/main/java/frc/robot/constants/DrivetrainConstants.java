package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DrivetrainConstants {

    public static final double maxSpeedMetersPerSecond = 4.0;
    public static final double maxAngularSpeed = Math.PI;

    public static final double directionSlewRate = 4.0; // rads/sec   - turning
    public static final double magnitudeSlewRate = 5.0; // percent/second (1 = 100%)   - forward/backward/traverse
    public static final double rotationalSlewRate = 12.0; // percent/second (1 = 100%)   - rotation

    public static final NetworkTable directionSlewNet = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("DirectionalSlewRate").setDouble(directionSlewRate);
    public static final NetworkTable magnitudeSlewNet = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("MagnitudeSlewRate").setDouble(magnitudeSlewRate);
    public static final NetworkTable rotationSlewNet = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("RotationSlewRate").setDouble(rotationalSlewRate);

    public static final double drivingSpeedScalar = -1.0;
    public static final double rotationSpeedScalar = -2.0;

    // TUNED
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
    );

    // FIXME: Right now 0degrees is in the Y-positive direction, when normally 0 rad is in the X-positive direction. Could this be an issue?
    // TUNED
    public static final double frontLeftChassisAngularOffset = 5.772;
    public static final double frontRightChassisAngularOffset = 6.099;
    public static final double rearLeftChassisAngularOffset = 0.871 + (Math.PI);
    public static final double rearRightChassisAngularOffset = 3.650;

    /**
     *     public static final double frontLeftChassisAngularOffset = 5.772 - (Math.PI/2) -(Math.PI/2);
     *     public static final double frontRightChassisAngularOffset = 6.09 - (Math.PI)+(0.0);
     *     public static final double rearLeftChassisAngularOffset = .871;
     *     public static final double rearRightChassisAngularOffset =  3.650 + (Math.PI/2) +(Math.PI/2);
     */
    // SPARK MAX CAN ID
    // TUNED
    public static final int frontLeftDrivingPort = 5;
    public static final int rearLeftDrivingPort = 1;
    public static final int frontRightDrivingPort = 7;
    public static final int rearRightDrivingPort = 3;

    public static final int frontLeftTurningPort = 6;
    public static final int rearLeftTurningPort = 2;
    public static final int frontRightTurningPort = 8;
    public static final int rearRightTurningPort = 18;

    public static final boolean gyroReversed = false; // TUNED
    public static final boolean turningEncoderReversed = true;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int drivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward.
    public static final double freeSpeedRpm = 5676.0; // from NEO datasheet, do not tune.
    public static final double drivingMotorFreeSpeedRps = freeSpeedRpm / 60.0;
    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters) / drivingMotorReduction;

    // Conversion factors (Revs -> Meters, RPM -> M/S).
    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0; // meters per second

    public static final double turningEncoderPositionFactor = 2 * Math.PI; // radians
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double turningEncoderPositionPIDMinInput = 0.0; // radians
    public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor; // radians

    // PIDs for driving and turning. minAndMaxOutputs are for SparkMax -1,1 setting, not voltage. TODO: These values must be tuned.
    public static final double drivingP = 0.06;
    public static final double drivingI = 0.0;
    public static final double drivingD = 0.0;
    public static final double drivingFF = 1.0 / driveWheelFreeSpeedRps;
    public static final double drivingMinOutput = -1.0;
    public static final double drivingMaxOutput = 1.0;

    public static final double turningP = 0.6;
    public static final double turningI = 0.0;
    public static final double turningD = 0.0;
    public static final double turningFF = 0.0;
    public static final double turningMinOutput = -1.0;
    public static final double turningMaxOutput = 1.0;

    // Idle mode for driving and turning motor
    public static final CANSparkMax.IdleMode drivingMotorIdleMode = CANSparkMax.IdleMode.kCoast;
    public static final CANSparkMax.IdleMode turningMotorIdleMode = CANSparkMax.IdleMode.kBrake;

    // Current limits for motors, set using smartcurrentlimits in swervemodulecontroller
    public static final int drivingMotorCurrentLimit = 40; // amps
    public static final int turningMotorCurrentLimit = 20; // amps
}
