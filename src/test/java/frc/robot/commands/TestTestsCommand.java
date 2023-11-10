package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;


import edu.wpi.first.wpilibj2.command.CommandBase;
import org.junit.jupiter.api.Test;

public class TestTestsCommand extends CommandBase {
    public TestTestsCommand() {}

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Test
    public void passing() {
        int x = 5;

        x = x + 5;
        assertEquals(x, 10);
    }

    @Test
    public  void fail() {
        int x = 5;
        x = x + 4;
        assertEquals(x, 10);
    }
}
