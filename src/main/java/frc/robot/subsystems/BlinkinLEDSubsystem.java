package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLEDSubsystem extends SubsystemBase {

    private final Spark blinkinDriver = new Spark(9);

    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final NetworkTableEntry ledNT = networkTableInstance.getTable("LEDs").getDouble("LEDValue");

    @Override
    public void periodic() {
        super.periodic();
        ledNT.setDouble(blinkinDriver.get());
    }

    public void setLED(double value) {
        blinkinDriver.set(value);
    }
}
