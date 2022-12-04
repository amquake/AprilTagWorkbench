package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionEstimation {

    private static final TargetModel tagModel = TargetModel.ofPlanarRect(
        Units.inchesToMeters(6),
        Units.inchesToMeters(6)
    );

    /**
     * Performs solvePNP using 3d-2d point correspondencies to estimate the field-to-camera transformation.
     * If only one tag is visible, the result may have an alternate solution.
     * 
     * <p><b>Note:</b> The returned transformation is from the field origin to the camera pose!
     * 
     * @param prop The camera properties
     * @param corners The visible tag corners in the 2d image
     * @param visibleTags The visible tags in 3d field space
     * @return The transformation that maps the field origin to the camera pose
     */
    public static PNPResults estimateTagsPNP(
            SimCamProperties prop, List<TargetCorner> corners, List<AprilTag> visibleTags) {
        if(visibleTags == null || corners == null ||
                corners.size() != visibleTags.size()*4 || visibleTags.size() == 0) {
            return new PNPResults(new Transform3d(), new Transform3d(), 0);
        }
        // single-tag pnp
        if(corners.size() == 4) {
            var camToTag = OpenCVHelp.solveTagPNP(prop, tagModel.cornerOffsets, corners);
            var bestPose = visibleTags.get(0).pose.transformBy(camToTag.best.inverse());
            var altPose = new Pose3d();
            if(camToTag.ambiguity != 0) altPose = visibleTags.get(0).pose.transformBy(camToTag.alt.inverse());
            var o = new Pose3d();
            return new PNPResults(
                new Transform3d(o, bestPose),
                new Transform3d(o, altPose),
                camToTag.ambiguity
            );
        }
        // multi-tag pnp
        else {
            var objectTrls = new ArrayList<Translation3d>();
            for(var tag : visibleTags) objectTrls.addAll(tagModel.getFieldCorners(tag.pose));
            var camToOrigin = OpenCVHelp.solveTagsPNP(prop, objectTrls, corners);
            return new PNPResults(camToOrigin.best.inverse(), new Transform3d(), 0);
        }
    }

    /**
     * The best estimated transformation to the target, and possibly an alternate transformation
     * depending on the solvePNP method. If an alternate solution is present, the ambiguity value
     * represents the ratio of reprojection error in the best solution to the alternate (best / alternate).
     */
    public static class PNPResults {
        public final Transform3d best;
        /**
         * Alternate, ambiguous solution from solvepnp. This may be empty
         * if no alternate solution is found.
         */
        public final Transform3d alt;
        /** If no alternate solution is found, this is 0 */
        public final double ambiguity;
        public PNPResults(Transform3d best, Transform3d alt, double ambiguity) {
            this.best = best;
            this.alt = alt;
            this.ambiguity = ambiguity;
        }
    }
}
