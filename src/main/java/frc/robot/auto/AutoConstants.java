package frc.robot.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class AutoConstants {
    
    // our maximum speeds/accelerations during auto
    public static final double kMaxLinearSpeed = Units.feetToMeters(13);
    public static final double kMaxLinearAcceleration = Units.feetToMeters(18);
    public static final double kMaxAngularSpeed = Units.rotationsToRadians(1.75);
    public static final double kMaxAngularAcceleration = Units.rotationsToRadians(3);

    public static final double kPXController = 3; // pose PID control. 1 meter error in x = kP meters per second in target x velocity 
    public static final double kPYController = 3;
    public static final double kPThetaController = 5;
    public static final double kDThetaController = 0.1;

    // constraints for the theta controller on velocity (omega) and acceleration (alpha)
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        Units.rotationsToRadians(1.5),
        Units.rotationsToRadians(3)
    );
    public static final double kThetaPositionTolerance = Units.degreesToRadians(3.5);
    public static final double kThetaVelocityTolerance = Units.degreesToRadians(10);

    // packaged configs for path following
    public static final TrajectoryConfig kFastSpeedConfig = new TrajectoryConfig(
        Units.feetToMeters(11), 
        Units.feetToMeters(14)
    );
    public static final TrajectoryConfig kMediumSpeedConfig = new TrajectoryConfig(
        Units.feetToMeters(9), 
        Units.feetToMeters(11)
    );
    public static final TrajectoryConfig kSlowSpeedConfig = new TrajectoryConfig(
        Units.feetToMeters(6), 
        Units.feetToMeters(6)
    );
}
