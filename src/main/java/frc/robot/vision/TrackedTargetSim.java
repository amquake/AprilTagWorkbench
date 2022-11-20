package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtils;

/**
 * A class associating a tracked april tag with its generated estimate of noise
 * based on its transform from the camera.
 * <br></br>
 * Provides utility methods for simulating tag detection noise.
 */
//TODO: delete
public final class TrackedTargetSim extends PhotonTrackedTarget{

    // targets with transform yaws greater than this should not be visible
    private static final Rotation2d kMaxTagToCamYaw = Rotation2d.fromDegrees(80);
    
    //public final Vector<N3> measurementStdDevsXY;

    public TrackedTargetSim(PhotonTrackedTarget target) {
        createFromPacket(target.populatePacket(new Packet(PACK_SIZE_BYTES)));
    }


    /**
     * Clone given targets and remove those that should not be visible to the camera.
     * @param targets
     */
    public static List<PhotonTrackedTarget> filterVisibleTags(List<PhotonTrackedTarget> targets) {
        List<PhotonTrackedTarget> list = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            Transform3d camToTag = target.getBestCameraToTarget();
            Rotation2d tagToCamYaw = new Rotation2d(
                camToTag.inverse().getX(), camToTag.inverse().getY());
            if(Math.abs(tagToCamYaw.getDegrees()) < kMaxTagToCamYaw.getDegrees()) {
                list.add(target);
            }
        }
        return list;
    }

    /**
     * Clone given targets and estimate their ambiguity based on their transformation from the camera.
     * @param targets
     */
    public static List<PhotonTrackedTarget> estimateTagAmbiguity(List<PhotonTrackedTarget> targets) {
        int debug = 0;
        List<PhotonTrackedTarget> list = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            Transform3d tagToCam = target.getBestCameraToTarget().inverse();
            Rotation2d tagToCamAngle = new Rotation2d(Math.hypot(
                new Rotation2d(tagToCam.getX(), tagToCam.getY()).getRadians(),
                new Rotation2d(tagToCam.getX(), tagToCam.getZ()).getRadians()
            ));

            double tagToCamDist = tagToCam.getTranslation().getNorm();

            // estimate ambiguity
            double angleAmbiguity = MathUtils.map(
                tagToCamAngle.getDegrees(), 8, 14, 
                0.2426 - 0.0394, 
                0
            );
            angleAmbiguity = MathUtils.clamp(angleAmbiguity, 0, 1);
            double distAmbiguity = MathUtils.map(tagToCamDist, 1.53, 2.335, 0.0288, 0.0394);
            distAmbiguity = MathUtils.clamp(distAmbiguity, 0, 1);

            if(debug==0) {
                SmartDashboard.putNumber("Sim/tagToCamX", tagToCam.getX());
                SmartDashboard.putNumber("Sim/tagToCamY", tagToCam.getY());
                SmartDashboard.putNumber("Sim/tagToCamZ", tagToCam.getZ());
                SmartDashboard.putNumber("Sim/tagToCamAngle", tagToCamAngle.getDegrees());
                SmartDashboard.putNumber("Sim/angleAmbiguity", angleAmbiguity);
                SmartDashboard.putNumber("Sim/distAmbiguity", distAmbiguity);
            }

            list.add(
                new PhotonTrackedTarget(
                    target.getYaw(),
                    target.getPitch(),
                    target.getArea(),
                    target.getSkew(),
                    target.getFiducialId(),
                    target.getBestCameraToTarget(),
                    target.getAlternateCameraToTarget(),
                    distAmbiguity + angleAmbiguity,
                    target.getCorners()
                )
            );
            debug++;
        }
        return list;
    }

    /**
     * Clone given targets and flip them to their alternate "ambiguous" pose possibility
     * using their ambiguity as a probabillity for flipping.
     * @param targets
     */
    public static List<PhotonTrackedTarget> estimateAmbiguityUncertainty(List<PhotonTrackedTarget> targets) {
        List<PhotonTrackedTarget> list = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            double probFlip = MathUtils.map(target.getPoseAmbiguity(), 0.05, 0.25, 0, 4.0 / 3.0 * 0.02);
            probFlip = MathUtils.clamp(probFlip, 0, 1);
            double prob = Math.random();
            boolean shouldFlip = prob < probFlip && target.getAlternateCameraToTarget() != null;
            //TODO: account for Z flipping as well and special case when parallel to the tag
            list.add(
                new PhotonTrackedTarget(
                    target.getYaw(),
                    target.getPitch(),
                    target.getArea(),
                    target.getSkew(),
                    target.getFiducialId(),
                    shouldFlip ? target.getAlternateCameraToTarget() : target.getBestCameraToTarget(),
                    shouldFlip ? target.getBestCameraToTarget() : target.getAlternateCameraToTarget(),
                    target.getPoseAmbiguity(),
                    target.getCorners()
                )
            );
        }
        return list;
    }

    /**
     * Clone given targets and estimate new transformations after applying noise.
     * @param targets
     */
    public static List<PhotonTrackedTarget> estimateTagNoise(List<PhotonTrackedTarget> targets) {
        List<PhotonTrackedTarget> list = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
        }
        return list;
    }
}
