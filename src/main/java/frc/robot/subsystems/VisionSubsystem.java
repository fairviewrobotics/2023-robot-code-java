package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.VisionUtils;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    public int cyclesVisible = 0;

    private final LinearFilter zFilter = LinearFilter.singlePoleIIR(1.0, 0.02);
    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(1.0, 0.02);

    public double mostRecentZ = 0.0;
    public double mostRecentX = 0.0;

    private final DoubleEntry ZSetpoint = NT("ZSetpoint");
    public final DoubleEntry ZOutput = NT("ZOutput");
    private final DoubleEntry ZMeasurement = NT("ZMeasurement");
    private final DoubleEntry ZUnfiltered = NT("ZUnfiltered");

    private final DoubleEntry ZP = NT("ZP");
    private final DoubleEntry ZI = NT("ZI");
    private final DoubleEntry ZD = NT("ZD");

    public final ProfiledPIDController lineupZPID = new ProfiledPIDController(
            VisionConstants.lineupZP,
            VisionConstants.lineupZI,
            VisionConstants.lineupZD,
            VisionConstants.lineupZConstraints
    );

    public final DoubleEntry XSetpoint = NT("XSetpoint");
    private final DoubleEntry XMeasurement = NT("XMeasurement");
    public final DoubleEntry XOutput = NT("XOutput");

    public final DoubleEntry TTASetpoint = NT("TTASetpoint");
    public final DoubleEntry TTAMeasurement = NT("TTAMeasurement");
    public final DoubleEntry TTAOutput = NT("TTAOutput");

    private DoubleEntry NT(String x) {
        return NetworkTableInstance.getDefault().getTable("Vision").getDoubleTopic(x).getEntry(0.0);
    }

    public void calculateAprilTagTargets() {
        var targets = VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Fiducials;
        if (targets.length > 0) {
            mostRecentX = xFilter.calculate(targets[0].getRobotPose_TargetSpaceDouble()[0]);
            ZUnfiltered.set(targets[0].getRobotPose_TargetSpaceDouble()[0]);
            mostRecentZ = zFilter.calculate(targets[0].getRobotPose_TargetSpaceDouble()[2]);
        }
    }

    public VisionSubsystem() {
        ZP.set(VisionConstants.lineupZP);
        ZI.set(VisionConstants.lineupZI);
        ZD.set(VisionConstants.lineupZD);
    }

    @Override
    public void periodic() {
        calculateAprilTagTargets();

        lineupZPID.setP(ZP.get());
        lineupZPID.setI(ZI.get());
        lineupZPID.setD(ZD.get());

        ZMeasurement.set(mostRecentZ);
        ZSetpoint.set(lineupZPID.getGoal().position);

        XMeasurement.set(mostRecentX);
    }
}
