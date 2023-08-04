package frc.robot.controllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivetrainConstants;

// NOTE: use encoders from cansparkmax or specify port?
// TODO: convert encoder rpm/rev counts to meters per second, degree, etc.
public class SwerveModuleController {
    private CANSparkMax drivingMotor;
    private CANSparkMax turningMotor;
    private RelativeEncoder drivingEncoder;
    private SparkMaxAbsoluteEncoder turningEncoder;
    private SparkMaxPIDController drivingPID;
    private SparkMaxPIDController turningPID;
    private SwerveModuleState m_desiredState;
    private double chassisAngularOffset;

    public SwerveModuleController(int drivingPort, int turningPort, double chassisAngularOffset_) {
        chassisAngularOffset = chassisAngularOffset_;
        drivingMotor = new CANSparkMax(drivingPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        drivingPID = drivingMotor.getPIDController();
        turningPID = turningMotor.getPIDController();

        m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

        // Reset settings to a known state, in case SparkMAXs were swapped out.
        drivingMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        // Measurement for PIDs
        drivingPID.setFeedbackDevice(drivingEncoder);
        turningPID.setFeedbackDevice(turningEncoder);

        // Converting revolutions and RPMs to meters and meters/second.
        drivingEncoder.setPositionConversionFactor(DrivetrainConstants.drivingEncoderPositionFactor);
        drivingEncoder.setVelocityConversionFactor(DrivetrainConstants.drivingEncoderVelocityFactor);

        turningEncoder.setPositionConversionFactor(DrivetrainConstants.turningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(DrivetrainConstants.turningEncoderVelocityFactor);

        // Output shaft rotates in the opposite direction of the steering motor in the module, this we need to invert.
        turningEncoder.setInverted(DrivetrainConstants.turningEncoderReversed);

        // Enable PID wrap around - going through 0 to get to the setpoint. Going from 350 degrees to 10 degrees will only
        // take 20 degrees of movement as opposed to 340.
        turningPID.setPositionPIDWrappingEnabled(true);
        turningPID.setPositionPIDWrappingMinInput(DrivetrainConstants.turningEncoderPositionPIDMinInput);
        turningPID.setPositionPIDWrappingMaxInput(DrivetrainConstants.turningEncoderPositionPIDMaxInput);

        // Set the PID gains. TODO: Needs to be tuned.
        drivingPID.setP(DrivetrainConstants.drivingP);
        drivingPID.setI(DrivetrainConstants.drivingI);
        drivingPID.setD(DrivetrainConstants.drivingD);
        drivingPID.setFF(DrivetrainConstants.drivingFF);
        drivingPID.setOutputRange(DrivetrainConstants.drivingMinOutput, DrivetrainConstants.drivingMaxOutput);

        // Set the pid gains. TODO: Needs to be tuned.
        turningPID.setP(DrivetrainConstants.turningP);
        turningPID.setI(DrivetrainConstants.turningI);
        turningPID.setD(DrivetrainConstants.turningD);
        turningPID.setFF(DrivetrainConstants.turningFF);
        turningPID.setOutputRange(DrivetrainConstants.turningMinOutput, DrivetrainConstants.turningMaxOutput);

        // Idle mode can either be brake (brings motors to quick stop) or coast (motors spin down at own, natural rate)
        drivingMotor.setIdleMode(DrivetrainConstants.drivingMotorIdleMode);
        turningMotor.setIdleMode(DrivetrainConstants.turningMotorIdleMode);

        // Setting a current limit that controllers will try to adjust for. NEOs have low internal resistance,
        // large spikes in voltage can damage motor and controller.
        drivingMotor.setSmartCurrentLimit(DrivetrainConstants.drivingMotorCurrentLimit);
        turningMotor.setSmartCurrentLimit(DrivetrainConstants.turningMotorCurrentLimit);

        // Ensures there will be no wacky turning movement on startup
        m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0.0);

        // Saves these settings to the motor controller itself, in case of brownout.
        drivingMotor.burnFlash();
        turningMotor.burnFlash();
    }

    /** Returns state (speed and angle) for the module. **/
    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    /** Returns **/
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    /** Sets desired state (speed and angle) for the module. **/
    public void setDesiredState(SwerveModuleState newState) {
        // Apply chassis angular offset to the directed state.
        Rotation2d chassisAngularOffsetRotation = new Rotation2d(chassisAngularOffset);
        Rotation2d correctedDesiredAngle = newState.angle.plus(chassisAngularOffsetRotation);
        SwerveModuleState correctedDesiredState = new SwerveModuleState(newState.speedMetersPerSecond, correctedDesiredAngle);

        // Optimizing helps avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

        // Setpoints for PIDs
        drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        m_desiredState = newState;
    }

    /** Zeroes all the SwerveModule encoders. **/
    public void resetEncoders() {
        drivingEncoder.setPosition(0.0);
    }
}
