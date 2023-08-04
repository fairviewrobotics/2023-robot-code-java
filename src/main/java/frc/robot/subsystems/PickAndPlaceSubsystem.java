package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.commands.SetPickAndPlacePosition; //throwing an issue here because commands aren't translated yet
import frc.robot.constants.ArmConstants;
import frc.robot.constants.CommandValues;
import frc.robot.constants.IntakeConstants;

public class PickAndPlaceSubsystem extends SubsystemBase {
    private CANSparkMax elevatorMotor = new CANSparkMax(ArmConstants.elevatorMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax elbowMotor = new CANSparkMax(ArmConstants.elbowMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax wristMotor = new CANSparkMax(ArmConstants.wristMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax intakeOneMotor = new CANSparkMax(ArmConstants.intakeMotorOneId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax intakeTwoMotor = new CANSparkMax(ArmConstants.intakeMotorTwoId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    private SparkMaxAbsoluteEncoder elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private DigitalInput forwardLimit = new DigitalInput(ArmConstants.topBreakerId);
    private DigitalInput reverseLimit = new DigitalInput(ArmConstants.bottomBreakerId);
    private boolean elevatorZeroed = false;
    //Is the carriage at the top of the elevator?
    private boolean topHit = !forwardLimit.get();
    // Is the carriage at the bottom of the elevator?
    private boolean bottomHit = !reverseLimit.get();
    // Nested Telemetry class to hold telemetry publishers and subscribers
    private static class Telemetry {
        static DoublePublisher elbowPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish();
        static DoublePublisher elbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVelocity").publish();
        static DoublePublisher elevatorPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPosition").publish();
        static DoublePublisher elevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVelocity").publish();
        static DoublePublisher elevatorVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVoltage").publish();
        static BooleanPublisher bottomHit = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("BottomHit").publish();
        static BooleanPublisher topHit = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("TopHit").publish();
        static DoublePublisher wristPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPosition").publish();
        static DoublePublisher wristVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristVoltage").publish();
        static DoublePublisher elbowVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVolts").publish();
        static DoublePublisher intakeVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("IntakeVolts").publish();
        static BooleanPublisher elevatorZeroed = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("ElevatorZeroed").publish();
        static NetworkTable table = NetworkTableInstance.getDefault().getTable("NtPnP positions");
        static DoubleSubscriber elevatorValue = table.getDoubleTopic("Elevator").subscribe(0.0);
        static DoubleSubscriber elbowValue = table.getDoubleTopic("Elbow").subscribe(0.0);
        static DoubleSubscriber wristValue = table.getDoubleTopic("Wrist").subscribe(0.0);
        static NetworkTable nt = NetworkTableInstance.getDefault().getTable("DriverControl");
        static BooleanPublisher cubeNT = nt.getBooleanTopic("Cube").publish(); // Both
        static BooleanPublisher middlePlaceNT = nt.getBooleanTopic("Middle Place").publish(); // Place
        static BooleanPublisher floorNT = nt.getBooleanTopic("Floor").publish(); // Place
        static BooleanPublisher chuteNT = nt.getBooleanTopic("Chute Pickup").publish(); // Pickup
        static BooleanPublisher pickupNT = nt.getBooleanTopic("Pickup").publish();
        // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
        static BooleanPublisher groundNT = nt.getBooleanTopic("Ground Pickup").publish(); // Pickup
        static BooleanPublisher coneNT = nt.getBooleanTopic("Cone").publish(); // Both
        static BooleanPublisher highPlaceNT = nt.getBooleanTopic("High Place").publish(); // Place
        static BooleanPublisher visionNT = nt.getBooleanTopic("Vision").publish();
        private Telemetry() {
        }
    }
    public PickAndPlaceSubsystem() {
        //intake
        elbowMotor.setSmartCurrentLimit(38);
        wristMotor.setSmartCurrentLimit(38);
        intakeOneMotor.setSmartCurrentLimit(28);
        intakeTwoMotor.setSmartCurrentLimit(28);
        elevatorMotor.setSmartCurrentLimit(38);
        elbowMotor.setInverted(ArmConstants.elbowMotorInverted);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wristMotor.setInverted(true);
        intakeTwoMotor.setInverted(false);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeOneMotor.setInverted(false);
        intakeTwoMotor.setInverted(true);
        wristMotor.setSmartCurrentLimit(IntakeConstants.pitchMotorCurrentLimit);
        intakeOneMotor.burnFlash();
        intakeTwoMotor.burnFlash();
        wristMotor.burnFlash();
        elbowMotor.burnFlash();
        elevatorMotor.burnFlash();
        // Initialize encoder conversion factors and other settings
        elbowEncoder.setPositionConversionFactor(2.0 * Math.PI);
        elbowEncoder.setVelocityConversionFactor((2.0 * Math.PI) / 60);
        elbowEncoder.setInverted(true);
        elevatorEncoder.setPositionConversionFactor(ArmConstants.elevatorEncoderPositionConversionFactor);
        elevatorEncoder.setVelocityConversionFactor(ArmConstants.elevatorEncoderVelocityConversionFactor);
        wristEncoder.setPositionConversionFactor(2.0 * Math.PI);
        wristEncoder.setVelocityConversionFactor((2.0 * Math.PI) / 60.0);
        wristEncoder.setInverted(false);
    }
    private double absoluteWristPosition = new Rotation2d(-wristEncoder.getPosition()).minus(new Rotation2d(ArmConstants.wristEncoderPosOffset)).getRadians();
    private Rotation2d elbowPositionFirst = new Rotation2d(elbowEncoder.getPosition()).plus(new Rotation2d(absoluteWristPosition));
    private double elbowPositionRadians = elbowPositionFirst.minus(new Rotation2d(ArmConstants.elbowEncoderPosOffset)).getRadians();
    private double elevatorPositionMeters = elevatorEncoder.getPosition();
    // Getters for calculated values
    public double getAbsoluteWristPosition() {
        return absoluteWristPosition;
    }
    public double getElbowPositionRadians() {
        return elbowPositionRadians;
    }
    public double getElevatorPositionMeters() {
        return elevatorPositionMeters;
    }
    // Setters for voltage control
    public void setIntakesVoltage(double x) {
        intakeOneMotor.setVoltage(x);
        intakeTwoMotor.setVoltage(x);
    }
    public void setWristVoltage(double x) {
        wristMotor.setVoltage(x);
    }
    public void setElbowVoltage(double x) {
        elbowMotor.setVoltage(x);
    }
    public void setElevatorVoltage(double x) {
        double elevatorVoltage = x;
        Telemetry.elevatorVoltage.set(elevatorVoltage);
        if (!elevatorZeroed) {
            elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage);
        } else if (bottomHit && x < 0.0) {
            elevatorMotor.setVoltage(0.0);
        } else if (topHit && x > 0.0) {
            elevatorMotor.setVoltage(0.0);
        } else {
            elevatorMotor.setVoltage(x);
        }
    }
    @Override
    public void periodic() {
        super.periodic();
        if (!elevatorZeroed) {
            elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage);
        }
        if(bottomHit)
        {
            if (!elevatorZeroed) {
                elevatorEncoder.setPosition(0.0);
            }
            elevatorZeroed = true;

        }
        else if(topHit)
        {
            //elevatorEncoder.position = ArmConstants.elevatorMaxHeight
        }

        

        // Telemetry setting.
        Telemetry.elevatorZeroed.set(elevatorZeroed);
        Telemetry.elbowPosition.set(elbowPositionRadians);
        Telemetry.elevatorPosition.set(elevatorEncoder.getPosition());
        Telemetry.elbowVelocity.set(elbowEncoder.getVelocity());
        Telemetry.bottomHit.set(bottomHit);
        Telemetry.wristVoltage.set(wristMotor.getBusVoltage());
        Telemetry.elbowVoltage.set(elbowMotor.getBusVoltage());

        Telemetry.intakeVoltage.set(intakeOneMotor.getBusVoltage());
        Telemetry.elevatorPosition.set(elevatorPositionMeters);
        Telemetry.elevatorVelocity.set(elevatorEncoder.getVelocity());
        Telemetry.wristPosition.set(absoluteWristPosition);

        Telemetry.cubeNT.set(CommandValues.cube);
        Telemetry.coneNT.set(CommandValues.cone);
        Telemetry.floorNT.set(CommandValues.floor);
        Telemetry.chuteNT.set(CommandValues.chute);
        Telemetry.pickupNT.set(CommandValues.pickup);
        Telemetry.middlePlaceNT.set(CommandValues.middlePlace);
        Telemetry.highPlaceNT.set(CommandValues.highPlace);
        Telemetry.groundNT.set(CommandValues.ground);

        Telemetry.visionNT.set(CommandValues.vision);
        // Rest of the periodic() method, see above
    }
    // Nested class removed, as Java does not support nested static classes.
    // Method "NTPnP" removed as well, as it is commented out in the Kotlin code.
    // End of class
}









