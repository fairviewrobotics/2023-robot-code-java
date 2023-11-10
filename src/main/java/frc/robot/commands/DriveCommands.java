package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.CommandValues;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.TrajectoryConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class StandardDrive extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier sideways;
    private final DoubleSupplier radians;
    private final boolean fieldRelative;
    private final boolean limited;

    private final NetworkTableEntry fieldRelativeFromButton = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("FieldOriented");

    public StandardDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier forward, DoubleSupplier sideways, DoubleSupplier radians, boolean fieldRelative, boolean limited) {
        this.swerveSubsystem = swerveSubsystem;
        this.forward = forward;
        this.sideways = sideways;
        this.radians = radians;
        this.fieldRelative = fieldRelative;
        this.limited = limited;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double forwardDesired = MathUtil.applyDeadband(forward.getAsDouble(), 0.06);
        double sidewaysDesired = MathUtil.applyDeadband(sideways.getAsDouble(), 0.06);
        double radiansDesired = MathUtil.applyDeadband(radians.getAsDouble(), 0.06);

        swerveSubsystem.drive(forwardDesired, sidewaysDesired, radiansDesired, fieldRelativeFromButton.getBoolean(false), limited);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0.0, 0.0, 0.0, true, true);
    }
}

public class TrajectoryDrive {
    public static Command create(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        PIDController thetaController = new PIDController(
                TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        NetworkTableEntry thetaControllerError = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("TurningError");

        thetaControllerError.setDouble(thetaController.getPositionError());

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::pose,
                DrivetrainConstants.driveKinematics,

                // Position controllers
                new PIDController(TrajectoryConstants.kPXController, 0.0, 0.0),
                new PIDController(TrajectoryConstants.kPYController, 0.0, 0.0),
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem
        );

        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        swerveControllerCommand,
                        new RunCommand(() -> swerveSubsystem.drive(0.0, 0.0, 0.0, false, false))
                ),
                new RunCommand(() -> {
                    System.out.println("-------------------------");
                    System.out.println(swerveSubsystem.pose.x);
                    System.out.println(swerveSubsystem.pose.y);
                    thetaControllerError.setDouble(thetaController.getPositionError());
                    System.out.println("-------------------------");
                })
        );
    }
}

public class TrajectoryDrivePathPlanner {
    public static Command create(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory trajectory, boolean isFirstPath) {
        PIDController thetaController = new PIDController(
                TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        NetworkTableEntry thetaControllerError = NetworkTableInstance.getDefault().getTable("Swerve").getEntry("TurningError2");

        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
                trajectory,
                swerveSubsystem::pose,
                DrivetrainConstants.driveKinematics,

                // Position controllers
                new PIDController(TrajectoryConstants.kPXController, 0.0, 0.0),
                new PIDController(TrajectoryConstants.kPYController, 0.0, 0.0),
                thetaController,
                swerveSubsystem::setModuleStates,
                false,
                swerveSubsystem
        );

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        swerveSubsystem.resetOdometry(trajectory.getInitialHolonomicPose());
                    }
                }),
                swerveControllerCommand,
                new RunCommand(() -> thetaControllerError.setDouble(thetaController.getPositionError())),
                new RunCommand(() -> swerveSubsystem.drive(0.0, 0.0, 0.0, true, false))
        );
    }
}