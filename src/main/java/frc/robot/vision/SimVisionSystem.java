/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtils;
import frc.robot.vision.SimVisionTarget.CameraTargetRelation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class SimVisionSystem {

    private final PhotonCameraSim camSim;
    public double maxSightRangeMeters = Double.MAX_VALUE;
    public final double minTargetArea;

    private Transform3d robotToCamera;
    private final TimeInterpolatableBuffer<Pose3d> robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
    private final TimeInterpolatableBuffer<Pose3d> camPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    private Map<String, Set<SimVisionTarget>> tgtList = new HashMap<>();

    private final Field2d dbgField;
    private final FieldObject2d dbgCamera;

    private double lastTime = Timer.getFPGATimestamp();
    private double msUntilNextFrame = 0;

    /**
     * Create a simulated vision system involving a camera and coprocessor mounted on a mobile robot
     * running PhotonVision, detecting one or more targets scattered around the field.
     *
     * @param camSim The simulated camera handle
     * @param robotToCamera Transform to move from the robot's position to the camera's mount position 
     * @param minTargetArea Minimum area percent that that the target should be before it's recognized as a
     *     target by the camera. Match this with your contour filtering settings in the PhotonVision
     *     GUI.
     */
    public SimVisionSystem(
            PhotonCameraSim camSim,
            Transform3d robotToCamera,
            double minTargetArea) {
        this.camSim = camSim;
        this.robotToCamera = robotToCamera;
        this.minTargetArea = minTargetArea;

        dbgField = new Field2d();
        var name = camSim.getCamera().name;
        SmartDashboard.putData(name + " Sim Field", dbgField);
        dbgCamera = dbgField.getObject(name + " Camera");
    }

    
    /**
     * Add a target on the field which your vision system is designed to detect. The PhotonCamera from
     * this system will report the location of the robot relative to the subset of these targets which
     * are visible from the given robot position.
     * 
     * <p>By default these are added under the type "targets".
     *
     * @param targets Targets to add to the simulated field
     */
    public void addVisionTargets(SimVisionTarget... targets) {
        addVisionTargets("targets", targets);
    }
    /**
     * Add a target on the field which your vision system is designed to detect. The PhotonCamera from
     * this system will report the location of the robot relative to the subset of these targets which
     * are visible from the given robot position.
     * 
     * <p>The AprilTags from this layout will be added as vision targets under "apriltags".
     * The poses added preserve the tag layout's current alliance origin.
     *
     * @param tagLayout The field tag layout to get Apriltag poses and IDs from 
     */
    public void addVisionTargets(AprilTagFieldLayout tagLayout) {
        for(AprilTag tag : tagLayout.getTags()){ 
            addVisionTargets("apriltags",
            new SimVisionTarget(
                tagLayout.getTagPose(tag.ID).get(), // preserve alliance rotation
                Units.inchesToMeters(6),
                tag.ID
            ));
        }
    }
    /**
     * Adds targets on the field which your vision system is designed to detect. The PhotonCamera from
     * this system will report the location of the robot relative to the subset of these targets which
     * are visible from the given robot position.
     * 
     * <p>By default these are added under the type "targets".
     * 
     * @param type Type of target (e.g. "cargo").
     * @param targets Targets to add to the simulated field
     */
    public void addVisionTargets(String type, SimVisionTarget... targets) {
        if(tgtList.get(type) == null) tgtList.put(type, new HashSet<>());
        for(var tgt : targets) {
            tgtList.get(type).add(tgt);
        }
    }
    public void clearVisionTargets() {
        tgtList.clear();
    }
    public Set<SimVisionTarget> removeVisionTargets(String type) {
        return tgtList.remove(type);
    }
    public Set<SimVisionTarget> removeVisionTargets(SimVisionTarget... targets) {
        var removeList = List.of(targets);
        var removedSet = new HashSet<SimVisionTarget>();
        for(var entry : tgtList.entrySet()) {
            entry.getValue().removeIf(t -> {
                if(removeList.contains(t)) {
                    removedSet.add(t);
                    return true;
                }
                else return false;
            });
        }
        return removedSet;
    }

    /**
     * Maximum distance at which your camera can detect the lighting of the target.
     * Note that minimum target area of the image is separate from this.
     * @param rangeMeters Maximum distance in meters the target is illuminated to the camera
     */
    public void setMaxSightRange(double rangeMeters) {
        this.maxSightRangeMeters = rangeMeters;
    }

    /**
     * Adjust the camera position relative to the robot. Use this if your camera is on a gimbal or
     * turret or some other mobile platform.
     *
     * @param robotToCamera New transform from the robot to the camera
     */
    public void adjustCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    /**
     * Get the simulated camera.
     */
    public PhotonCameraSim getCameraSim() {
        return camSim;
    }

    /**
     * Get the robot pose in meters saved by the vision system secondsAgo.
     * @param secondsAgo Seconds to look into the past
     */
    public Pose3d getRobotPose(double secondsAgo) {
        return robotPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(new Pose3d());
    }
    /**
     * Get the camera pose in meters saved by the vision system secondsAgo.
     * @param secondsAgo Seconds to look into the past
     */
    public Pose3d getCameraPose(double secondsAgo) {
        return camPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(new Pose3d());
    }

    /**
     * Periodic update. Ensure this is called repeatedly-- camera performance is used to
     * automatically determine if a new frame should be submitted.
     * @param robotPoseMeters The current robot pose in meters
     */
    public void update(Pose2d robotPoseMeters) {
        update(new Pose3d(robotPoseMeters));
    }
    /**
     * Periodic update. Ensure this is called repeatedly-- camera performance is used to
     * automatically determine if a new frame should be submitted.
     * @param robotPoseMeters The current robot pose in meters
     */
    public void update(Pose3d robotPoseMeters) {
        var targetTypes = tgtList.entrySet();
        // update vision targets on field
        targetTypes.forEach(entry -> dbgField.getObject(entry.getKey()).setPoses(
            entry.getValue().stream().map(t -> t.getPose().toPose2d()).collect(Collectors.toList())
        ));

        if(robotPoseMeters == null) return;

        // check if this camera is ready for another frame update
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        double latencyMillis;

        robotPoseBuffer.addSample(now, robotPoseMeters);
        camPoseBuffer.addSample(now, robotPoseMeters.transformBy(robotToCamera));

        if(dt >= msUntilNextFrame/1000.0) {
            latencyMillis = camSim.prop.estLatencyMs();
            msUntilNextFrame = camSim.prop.estMsUntilNextFrame();
            lastTime = now;
        }
        else {
            return;
        }
        // poses latency milliseconds ago
        Pose3d robotPose = robotPoseBuffer.getSample(now - latencyMillis / 1000.0).get();
        Pose3d cameraPose = camPoseBuffer.getSample(now - latencyMillis / 1000.0).get();

        dbgField.setRobotPose(robotPose.toPose2d());
        dbgCamera.setPose(cameraPose.toPose2d());

        ArrayList<PhotonTrackedTarget> visibleTgtList = new ArrayList<>(tgtList.size());
        
        // update camera's visible targets
        targetTypes.forEach((entry) -> entry.getValue().forEach((tgt) -> {
            // pose isn't visible, skip to next
            if(!canSeeTargetPose(cameraPose, tgt)) return;

            // various helper geometries between camera and target
            var rel = new CameraTargetRelation(cameraPose, tgt.getPose());

            // find target's 3d corner points
            var fieldCorners = tgt.getFieldCorners();
            if(!tgt.getModel().isPlanar) fieldCorners = tgt.getAgnosticFieldCorners(cameraPose);
            // project 3d target points into 2d image points
            var targetCorners = OpenCVHelp.projectPoints(
                cameraPose,
                camSim.prop,
                fieldCorners
            );
            // estimate pixel noise
            targetCorners = camSim.prop.estPixelNoise(targetCorners);
            // find contour area
            double areaPixels = OpenCVHelp.getPolygonArea(targetCorners);
            double areaPercent = areaPixels / camSim.prop.getResArea() * 100;

            // projected target can't be detected, skip to next
            if(!canSeeCorners(areaPercent, targetCorners)) return;

            var pnpSim = new OpenCVHelp.PNPResults(new Transform3d(), new Transform3d(), 0);
            // only do 3d estimation if we have a planar target
            if(tgt.getModel().isPlanar) {
                pnpSim = OpenCVHelp.solvePNP(camSim.prop, tgt.getModel().cornerOffsets, targetCorners);
            }

            visibleTgtList.add(
                new PhotonTrackedTarget(
                    rel.camToTargYaw.getDegrees(),
                    rel.camToTargPitch.getDegrees(),
                    areaPercent,
                    0.0,
                    tgt.id,
                    pnpSim.best,
                    pnpSim.alt,
                    pnpSim.ambiguity,
                    List.of(targetCorners)
                )
            );
        }));

        camSim.submitProcessedFrame(latencyMillis, visibleTgtList);
    }

    /**
     * Determines if this target's pose should be visible to the camera without considering
     * its projected image points. Does not account for image area.
     * @param camPose Camera's 3d pose
     * @param target Vision target containing pose and shape
     * @return If this vision target can be seen before image projection.
     */
    public boolean canSeeTargetPose(Pose3d camPose, SimVisionTarget target) {
        var rel = new CameraTargetRelation(camPose, target.getPose());
        boolean canSee = (
            // target translation is outside of camera's FOV
            (Math.abs(rel.camToTargYaw.getDegrees()) < camSim.prop.getHorizFOV().getDegrees() / 2) &&
            (Math.abs(rel.camToTargPitch.getDegrees()) < camSim.prop.getVertFOV().getDegrees() / 2) &&
            // camera is behind planar target and it should be occluded
            (!target.getModel().isPlanar || Math.abs(rel.targToCamAngle.getDegrees()) < 90) &&
            // target is too far
            (rel.camToTarg.getTranslation().getNorm() <= maxSightRangeMeters)
        );
        return canSee;
    }
    /**
     * Determines if this target can be detected after image projection.
     * @param areaPercent The target contour's area percentage of the image
     * @param corners The corners of the target as image points(x,y)
     * @return If the target area is large enough and the corners are inside
     *     the camera's FOV
     * @see OpenCVHelp#projectPoints(Pose3d, SimCamProperties, Translation3d...)
     * @see OpenCVHelp#getPolygonArea(TargetCorner...)
     */
    public boolean canSeeCorners(double areaPercent, TargetCorner... corners) {
        // corner is outside of resolution
        for(var corner : corners) {
            if(!MathUtils.within(corner.x, 0, camSim.prop.getResWidth()) ||
                    !MathUtils.within(corner.y, 0, camSim.prop.getResHeight())) {
                return false;
            }
        }
        // target too small
        return areaPercent >= minTargetArea;
    }
}
