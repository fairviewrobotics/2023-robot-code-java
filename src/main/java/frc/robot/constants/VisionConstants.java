package frc.robot.constants;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class VisionConstants {
    public enum Pipelines {
        APRILTAG(0),
        CONE(1),
        RETROREFLECTIVE(2),
        CUBE(3);

        private final int value;

        Pipelines(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public static final double lineupZP = 0.4;
    public static final double lineupZI = 0.0;
    public static final double lineupZD = 0.0;

    public static final TrapezoidProfile.Constraints lineupZConstraints = new TrapezoidProfile.Constraints(0.75, 0.5);

    public static final double lineupRotP = 0.5;
    public static final double lineupRotI = 0.0;
    public static final double lineupRotD = 0.0;

    public static final TrapezoidProfile.Constraints lineupRotConstraints = new TrapezoidProfile.Constraints(0.75, 0.2);

    public static final double retroreflectiveP = 0.03;
    public static final double retroreflectiveI = 0.0;
    public static final double retroreflectiveD = 0.0;

    public static final double ttaP = 5.0;
    public static final double ttaI = 0.0;
    public static final double ttaD = 0.0;
    public static final TrapezoidProfile.Constraints ttaConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    public static final ObjectMapper mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    private VisionConstants() {
        // private constructor to prevent instantiation
    }
}
