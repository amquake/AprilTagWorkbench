package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class MathUtils {
    public static Pose3d calcAvg(Pose3d... values){
        if(values.length == 0) return new Pose3d();
        Vector<N3> avgAxis = VecBuilder.fill(
            Arrays.stream(values).mapToDouble(pose -> pose.getRotation().getAxis().get(0, 0)).average().orElse(0),
            Arrays.stream(values).mapToDouble(pose -> pose.getRotation().getAxis().get(1, 0)).average().orElse(0),
            Arrays.stream(values).mapToDouble(pose -> pose.getRotation().getAxis().get(2, 0)).average().orElse(0)
        );
        double avgAngle = Arrays.stream(values).mapToDouble(pose -> pose.getRotation().getAngle()).average().orElse(0);
        Rotation3d averageRot = new Rotation3d(
            avgAxis,
            avgAngle
        );
        return new Pose3d(
            calcAvg(Arrays.stream(values).mapToDouble(pose -> pose.getX()).toArray()),
            calcAvg(Arrays.stream(values).mapToDouble(pose -> pose.getY()).toArray()),
            calcAvg(Arrays.stream(values).mapToDouble(pose -> pose.getZ()).toArray()),
            averageRot
        );
    }
    public static double calcAvg(double... values){
        if(values.length == 0) return 0;
        return Arrays.stream(values).average().getAsDouble();
    }

    public static Pose3d calcStdDev(Pose3d... values){
        if(values.length == 0) return new Pose3d();
        Pose3d avgPose = calcAvg(values);

        double xAxisDev = 0;
        double yAxisDev = 0;
        double zAxisDev = 0;
        double angleDev = 0;
        for(Pose3d v : values){
            xAxisDev += Math.pow(
                v.getRotation().getAxis().get(0, 0) - avgPose.getRotation().getAxis().get(0, 0),
                2
            );
            yAxisDev += Math.pow(
                v.getRotation().getAxis().get(1, 0) - avgPose.getRotation().getAxis().get(1, 0),
                2
            );
            zAxisDev += Math.pow(
                v.getRotation().getAxis().get(2, 0) - avgPose.getRotation().getAxis().get(2, 0),
                2
            );
            angleDev += Math.pow(
                v.getRotation().getAngle() - avgPose.getRotation().getAngle(),
                2
            );
        }
        xAxisDev /= values.length;
        xAxisDev = Math.sqrt(xAxisDev);
        yAxisDev /= values.length;
        yAxisDev = Math.sqrt(yAxisDev);
        zAxisDev /= values.length;
        zAxisDev = Math.sqrt(zAxisDev);
        angleDev /= values.length;
        angleDev = Math.sqrt(angleDev);

        return new Pose3d(
            calcStdDev(Arrays.stream(values).mapToDouble(pose -> pose.getX()).toArray()),
            calcStdDev(Arrays.stream(values).mapToDouble(pose -> pose.getY()).toArray()),
            calcStdDev(Arrays.stream(values).mapToDouble(pose -> pose.getZ()).toArray()),
            new Rotation3d(
                VecBuilder.fill(xAxisDev, yAxisDev, zAxisDev),
                angleDev
            )
        );
    }
    public static double calcStdDev(double... values){
        if(values.length == 0) return 0;
        double mean = calcAvg(values);
        double variance = 0;
        for(double v : values){
            variance += Math.pow((v - mean), 2);
        }
        variance /= values.length;
        return Math.sqrt(variance);
    }

    public static boolean within(int value, int from, int to){
        return from <= value && value <= to;
    }
    public static boolean within(double value, double from, double to){
        return from <= value && value <= to;
    }
    public static int clamp(int value, int from, int to){
        return Math.max(from, Math.min(value, to));
    }
    public static double clamp(double value, double from, double to){
        return Math.max(from, Math.min(value, to));
    }

    /**
     * Linear percentage from start to end
     * @return Percentage (NOT CLAMPED)
     */
    public static double percentTo(double value, double start, double end) {
        return (value-start)/(end-start);
    }
    /**
     * Linearly interpolates from start to end
     * @return Resulting value (NOT CLAMPED)
     */
    public static double lerp(double percent, double start, double end){
        return start+(end-start)*percent;
    }
    /**
     * Linearly maps value in [startFrom, startTo] to [endFrom, endTo]
     * @return Resulting value (NOT CLAMPED)
     */
    public static double map(double value, double inStart, double inEnd, double outStart, double outEnd) {
        return lerp(percentTo(value, inStart, inEnd), outStart, outEnd);
    }
}
