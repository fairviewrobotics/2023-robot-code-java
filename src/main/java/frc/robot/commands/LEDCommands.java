package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.CommandValues;
import frc.robot.subsystems.BlinkinLEDSubsystem;

public class SetLEDValueConeCube extends CommandBase {
    private final BlinkinLEDSubsystem subsystem;

    public SetLEDValueConeCube(BlinkinLEDSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (CommandValues.auto) {
            if (CommandValues.balanced) {
                subsystem.setLED(0.59);
                System.out.println("Balanced");
            } else if (CommandValues.balancing) {
                subsystem.setLED(-0.31);
                System.out.println("Balancing");
            } else {
                subsystem.setLED(-0.29);
                System.out.println("Auto LEDs");
            }
        } else if (CommandValues.visionIsMovingRobot) {
            subsystem.setLED(-0.93);
        } else {
            if (CommandValues.cube) {
                subsystem.setLED(0.91); // 0.91
            } else if (!CommandValues.cube) {
                subsystem.setLED(0.69);
            } else {
                subsystem.setLED(0.93);
            }
        }
    }
}

public class SetLEDValue extends CommandBase {
    private final BlinkinLEDSubsystem subsystem;
    private final double value;

    public SetLEDValue(BlinkinLEDSubsystem subsystem, double value) {
        this.subsystem = subsystem;
        this.value = value;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setLED(value);
    }
}
