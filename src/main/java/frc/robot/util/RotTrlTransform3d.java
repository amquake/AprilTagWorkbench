package frc.robot.util;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents a transform that first rotates the pose around the origin,
 * and then translates it.
 */
public class RotTrlTransform3d {
    private final Translation3d trl;
    private final Rotation3d rot;

    public RotTrlTransform3d() {
        this(new Transform3d());
    }
    /**
     * The rotation-translation transformation that makes poses in the world relative to
     * this pose.
     * @param pose The new origin
     */
    public RotTrlTransform3d(Pose3d pose) {
        // rotate by change in origin rotation
        var inverseRot = pose.getRotation().unaryMinus();
        this.rot = inverseRot;
        // orientation is now aligned, translate to match origins
        this.trl = pose.getTranslation()
            .rotateBy(inverseRot)
            .unaryMinus();
    }
    public RotTrlTransform3d(Transform3d trf) {
        this(trf.getRotation(), trf.getTranslation());
    }
    public RotTrlTransform3d(Rotation3d rot, Translation3d trl) {
        this.rot = rot;
        this.trl = trl;
    }
    
    public Transform3d getTransform() {return new Transform3d(trl, rot);}
    public Translation3d getTranslation() {return trl;}
    public Rotation3d getRotation() {return rot;}
    public Translation3d apply(Translation3d trl) {
        return apply(new Pose3d(trl, new Rotation3d())).getTranslation();
    };
    public List<Translation3d> applyTrls(List<Translation3d> trls) {
        return trls.stream().map(t -> apply(t)).collect(Collectors.toList());
    }
    public Pose3d apply(Pose3d pose) {
        return new Pose3d(
            pose.getTranslation().rotateBy(rot).plus(trl),
            pose.getRotation().plus(rot)
        );
    }
    public List<Pose3d> applyPoses(List<Pose3d> poses) {
        return poses.stream().map(p -> apply(p)).collect(Collectors.toList());
    }
}
