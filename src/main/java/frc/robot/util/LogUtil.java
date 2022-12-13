package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public final class LogUtil {
    public static double[] toPoseArray(Pose2d pose) {
        return new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }
    public static double[] toPoseArray(Pose3d pose) {
        var rot = pose.getRotation();
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            rot.getQuaternion().getW(),
            rot.getQuaternion().getX(),
            rot.getQuaternion().getY(),
            rot.getQuaternion().getZ()
        };
    }
}
