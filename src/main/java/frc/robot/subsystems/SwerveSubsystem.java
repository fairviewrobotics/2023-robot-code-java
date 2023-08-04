package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
//import edu.wpi.first.networktables.NetworkTableValueConverter;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.controllers.SwerveModuleController;
import frc.robot.utils.NetworkTableUtils;
import frc.robot.utils.SwerveUtils;

public class SwerveSubsystem extends SubsystemBase {
    // Defining Motors
    private final SwerveModuleController frontLeft = new SwerveModuleController(
            DrivetrainConstants.frontLeftDrivingPort,
            DrivetrainConstants.frontLeftTurningPort,
            DrivetrainConstants.frontLeftChassisAngularOffset
    );

    private final SwerveModuleController frontRight = new SwerveModuleController(
            DrivetrainConstants.frontRightDrivingPort,
            DrivetrainConstants.frontRightTurningPort,
            DrivetrainConstants.frontRightChassisAngularOffset
    );

    private final SwerveModuleController rearLeft = new SwerveModuleController(
            DrivetrainConstants.rearLeftDrivingPort,
            DrivetrainConstants.rearLeftTurningPort,
            DrivetrainConstants.rearLeftChassisAngularOffset
    );

    private final SwerveModuleController rearRight = new SwerveModuleController(
            DrivetrainConstants.rearRightDrivingPort,
            DrivetrainConstants.rearRightTurningPort,
            DrivetrainConstants.rearRightChassisAngularOffset
    );

    // Gyro
    private final AHRS gyro = new AHRS();

    // Slew Rate Constants
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate Limiters
    private final SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DrivetrainConstants.magnitudeSlewRate);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(DrivetrainConstants.rotationalSlewRate);

    // Slew Rate Time
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    // Limelight Network Table
    private final NetworkTableUtils limelightTable = new NetworkTableUtils("limelight");

    // Convert Gyro angle to radians(-2pi to 2pi)
    private double heading() {
        return Units.degreesToRadians(-1 * (gyro.getAngle() + 180.0)) % (2.0 * Math.PI);
    }

    // Swerve Odometry
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            DrivetrainConstants.driveKinematics,
            Rotation2d.fromRadians(heading()),
            new SwerveModulePosition[]{
                    frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()
            }
    );

    // Network Tables Telemetry
    private final NetworkTableEntry setpointsTelemetry = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("Setpoints");
    private final NetworkTableEntry actualTelemetry = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("Actual");
    private final NetworkTableEntry poseTelemetry = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("Pose");
    private final NetworkTableEntry gyroHeading = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("GyroHeading");
    private final NetworkTableEntry frontrightpos = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("frpos");
    private final NetworkTableEntry frontleftpos = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("flpos");

    // Periodic
    @Override
    public void periodic() {
        // Update odometry
        odometry.update(
                Rotation2d.fromRadians(heading()),
                new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()}
        );

        frontrightpos.setDouble(frontRight.getPosition().getAngle().getRadians());
        frontleftpos.setDouble(frontLeft.getPosition().getAngle().getRadians());

        // Set Network Tables Telemetry
        NetworkTableValue setpointsValue = NetworkTableValueConverter.fromArray(new double[]{
                frontLeft.m_desiredState.getAngle().getRadians(), frontLeft.m_desiredState.getSpeedMetersPerSecond(),
                frontRight.m_desiredState.getAngle().getRadians(), frontRight.m_desiredState.getSpeedMetersPerSecond(),
                rearLeft.m_desiredState.getAngle().getRadians(), rearLeft.m_desiredState.getSpeedMetersPerSecond(),
                rearRight.m_desiredState.getAngle().getRadians(), rearRight.m_desiredState.getSpeedMetersPerSecond()
        });
        setpointsTelemetry.setDoubleArray(NetworkTableType.kDouble, setpointsValue);

        NetworkTableValue actualValue = NetworkTableValueConverter.fromArray(new double[]{
                frontLeft.getPosition().getAngle().getRadians(), frontLeft.getState().getSpeedMetersPerSecond(),
                frontRight.getPosition().getAngle().getRadians(), frontRight.getState().getSpeedMetersPerSecond(),
                rearLeft.getPosition().getAngle().getRadians(), rearLeft.getState().getSpeedMetersPerSecond(),
                rearRight.getPosition().getAngle().getRadians(), rearRight.getState().getSpeedMetersPerSecond()
        });
        actualTelemetry.setDoubleArray(NetworkTableType.kDouble, actualValue);

        Pose2d pose = odometry.getPoseMeters();
        NetworkTableValue poseValue = NetworkTableValueConverter.fromArray(new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()});
        poseTelemetry.setDoubleArray(NetworkTableType.kDouble, poseValue);

        gyroHeading.setDouble(heading());
    }

    // Define robot pose
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Reset odometry function
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromRadians(heading()),
                new SwerveModuleState[]{frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()},
                pose
        );
    }

    // Drive function - slew rate limited to prevent shearing of wheels
    public void drive(double forwardMetersPerSecond, double sidewaysMetersPerSecond, double radiansPerSecond, boolean fieldRelative, boolean rateLimit) {
        // Forward is xspeed, sideways is yspeed
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            double inputTranslationDirection = Math.atan2(sidewaysMetersPerSecond, forwardMetersPerSecond);
            double inputTranslationMagnitude = Math.sqrt(forwardMetersPerSecond * forwardMetersPerSecond + sidewaysMetersPerSecond * sidewaysMetersPerSecond);

            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DrivetrainConstants.directionSlewRate / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // super high number means slew is instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;

            double angleDifference = SwerveUtils.AngleDifference(inputTranslationDirection, currentTranslationDirection);
            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                        currentTranslationDirection,
                        inputTranslationDirection,
                        directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude > 1e-4) { // small number avoids floating-point errors
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                        currentTranslationDirection,
                        inputTranslationDirection,
                        directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            }

            previousTime = currentTime;

            xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(radiansPerSecond);
        } else {
            xSpeedCommanded = forwardMetersPerSecond;
            ySpeedCommanded = sidewaysMetersPerSecond;
            currentRotation = radiansPerSecond;
        }

        double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.maxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.maxSpeedMetersPerSecond;
        double rotationDelivered = currentRotation * DrivetrainConstants.maxAngularSpeed;

        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, Rotation2d.fromRadians(heading()))
            );
        } else {
            swerveModuleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Sets the wheels to an X configuration
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    }

    // Sets the wheels to a zeroed configuration
    public void setZero() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
        rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    // Resets Gyro
    public void zeroGyro() {
        gyro.reset();
    }

    // Resets Gyro and odometry
    public void zeroGyroAndOdometry() {
        gyro.reset();
        resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    }

    // Sets states of swerve modules
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    // Resets Swerve encoders
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
    }

    // Returns turn rate of the robot
    public double turnRate() {
        double coefficient = DrivetrainConstants.gyroReversed ? -1.0 : 1.0;
        return gyro.getRate() * coefficient;
    }
}
