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

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonVersion;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import frc.robot.vision.SimVisionTarget.CameraTargetRelation;

@SuppressWarnings("unused")
public class PhotonCameraSim {
    private final PhotonCamera cam;
    private final NetworkTableEntry latencyMillisEntry;
    private final NetworkTableEntry hasTargetEntry;
    private final NetworkTableEntry targetPitchEntry;
    private final NetworkTableEntry targetYawEntry;
    private final NetworkTableEntry targetAreaEntry;
    private final NetworkTableEntry targetSkewEntry;
    private final NetworkTableEntry targetPoseEntry;
    private final NetworkTableEntry versionEntry;

    /**
     * This simulated camera's {@link SimCamProperties}
     */
    public final SimCamProperties prop;
    private double lastTime = Timer.getFPGATimestamp();
    private double msUntilNextFrame = 0;
    
    private double maxSightRangeMeters = Double.MAX_VALUE;
    private double minTargetAreaPx = 100;

    private Transform3d robotToCamera;
    private final TimeInterpolatableBuffer<Pose3d> camPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    private final Field2d dbgCorners = new Field2d();
    
    /**
     * Constructs a handle for simulating {@link PhotonCamera} values.
     * Processing simulated targets through this class will change the associated
     * PhotonCamera's results.
     * 
     * <p><b>This constructor's camera has a 90 deg FOV with no simulated lag!</b>
     * 
     * <p>By default, the minimum target area is 100 pixels and there is no maximum sight range.
     *
     * @param camera The camera to be simulated
     * @param robotToCamera Transform in meters to move from the robot's origin to the camera's pose
     */
    public PhotonCameraSim(PhotonCamera camera, Transform3d robotToCamera) {
        this(camera, SimCamProperties.PERFECT_90DEG, robotToCamera);
    }
    /**
     * Constructs a handle for simulating {@link PhotonCamera} values.
     * Processing simulated targets through this class will change the associated
     * PhotonCamera's results.
     * 
     * <p>By default, the minimum target area is 100 pixels and there is no maximum sight range.
     *
     * @param camera The camera to be simulated
     * @param prop Properties of this camera such as FOV and FPS
     * @param robotToCamera Transform in meters to move from the robot's origin to the camera's pose
     */
    public PhotonCameraSim(PhotonCamera camera, SimCamProperties prop, Transform3d robotToCamera) {
        this.cam = camera;
        this.prop = prop;
        this.robotToCamera = robotToCamera;
        
        var rootTable = camera.rootTable;
        latencyMillisEntry = rootTable.getEntry("latencyMillis");
        hasTargetEntry = rootTable.getEntry("hasTargetEntry");
        targetPitchEntry = rootTable.getEntry("targetPitchEntry");
        targetYawEntry = rootTable.getEntry("targetYawEntry");
        targetAreaEntry = rootTable.getEntry("targetAreaEntry");
        targetSkewEntry = rootTable.getEntry("targetSkewEntry");
        targetPoseEntry = rootTable.getEntry("targetPoseEntry");
        versionEntry = camera.versionEntry;
        // Sets the version string so that it will always match the current version
        versionEntry.setString(PhotonVersion.versionString);
    }
    /**
     * Constructs a handle for simulating {@link PhotonCamera} values.
     * Processing simulated targets through this class will change the associated
     * PhotonCamera's results.
     *
     * @param camera The camera to be simulated
     * @param prop Properties of this camera such as FOV and FPS
     * @param robotToCamera Transform in meters to move from the robot's origin to the camera's pose
     * @param minTargetAreaPx The minimum number of pixels a detected target must take up in the camera's
     *     image to be processed. Match this with your contour filtering settings in the PhotonVision
     *     GUI. (pixels = percent / 100 * resolution area)
     * @param maxSightRangeMeters Maximum distance at which the target is illuminated to your camera.
     *     Note that minimum target area of the image is separate from this.
     */
    public PhotonCameraSim(
            PhotonCamera camera, SimCamProperties prop, Transform3d robotToCamera,
            double minTargetAreaPx, double maxSightRangeMeters) {
        this(camera, prop, robotToCamera);
        this.minTargetAreaPx = minTargetAreaPx;
        this.maxSightRangeMeters = maxSightRangeMeters;
    }

    public PhotonCamera getCamera() {
        return cam;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }
    /**
     * Get the camera pose in meters saved by the vision system secondsAgo.
     * @param secondsAgo Seconds to look into the past
     */
    public Pose3d getCameraPose(double secondsAgo) {
        return camPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(new Pose3d());
    }

    public double getMinTargetAreaPx() {
        return minTargetAreaPx;
    }
    public double getMinTargetAreaPercent() {
        return minTargetAreaPx / prop.getResArea() * 100;
    }
    public double getMaxSightRangeMeters() {
        return maxSightRangeMeters;
    }

    public Field2d getDebugCorners() {
        return dbgCorners;
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
            (Math.abs(rel.camToTargYaw.getDegrees()) < prop.getHorizFOV().getDegrees() / 2) &&
            (Math.abs(rel.camToTargPitch.getDegrees()) < prop.getVertFOV().getDegrees() / 2) &&
            // camera is behind planar target and it should be occluded
            (!target.getModel().isPlanar || Math.abs(rel.targToCamAngle.getDegrees()) < 90) &&
            // target is too far
            (rel.camToTarg.getTranslation().getNorm() <= maxSightRangeMeters)
        );
        return canSee;
    }
    /**
     * Determines if this target can be detected after image projection.
     * @param areaPixels The target contour's area in pixels of the image
     * @param corners The corners of the target as image points(x,y)
     * @return If the target area is large enough and the corners are inside
     *     the camera's FOV
     * @see OpenCVHelp#projectPoints(Pose3d, SimCamProperties, Translation3d...)
     * @see OpenCVHelp#getContourAreaPx(TargetCorner...)
     */
    public boolean canSeeCorners(double areaPixels, TargetCorner... corners) {
        // corner is outside of resolution
        for(var corner : corners) {
            if(MathUtil.clamp(corner.x, 0, prop.getResWidth()) != corner.x ||
                    MathUtil.clamp(corner.y, 0, prop.getResHeight()) != corner.y) {
                return false;
            }
        }
        // target too small
        return areaPixels >= minTargetAreaPx;
    }

    /**
     * Determine if this camera should process a new frame based on performance metrics and the time
     * since the last update. This returns an Optional which is either empty if no update should occur
     * or a Double of the latency in milliseconds of the frame which should be processed. If a
     * latency is returned, the last frame update time becomes the current time.
     * @return Optional double which is empty while blocked or the latency in milliseconds if ready
     */
    public Optional<Double> getShouldProcess() {
        // check if this camera is ready for another frame update
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        double latencyMillis;

        if(dt >= msUntilNextFrame/1000.0) {
            latencyMillis = prop.estLatencyMs();
            msUntilNextFrame = prop.estMsUntilNextFrame();
            lastTime = now;
            return Optional.of(latencyMillis);
        }
        else {
            // this camera isnt ready to process yet
            return Optional.empty();
        }
    }

    /**
     * The minimum number of pixels a detected target must take up in the camera's image
     * to be processed.
     */
    public void setMinTargetAreaPx(double areaPx) {
        this.minTargetAreaPx = areaPx;
    }
    /**
     * Maximum distance at which the target is illuminated to your camera.
     * Note that minimum target area of the image is separate from this.
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
     * Update the current camera pose given the current robot pose in meters.
     * This is dependent on the robot-to-camera transform, and camera poses are saved over time.
     */
    public void updateCameraPose(Pose3d robotPoseMeters) {
        camPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPoseMeters.transformBy(robotToCamera));
    }
    public void logDebugCorners(String table) {
        SmartDashboard.putData(table+"/"+cam.name+"-cam corners", dbgCorners);
    }

    public List<PhotonTrackedTarget> process(
            double latencyMillis, Pose3d cameraPose, List<SimVisionTarget> targets) {
        var visibleTgts = new ArrayList<PhotonTrackedTarget>();
        for(var tgt : targets) {
            // pose isn't visible, skip to next
            if(!canSeeTargetPose(cameraPose, tgt)) continue;

            // find target's 3d corner points
            var fieldCorners = tgt.getFieldCorners();
            if(!tgt.getModel().isPlanar) fieldCorners = tgt.getAgnosticFieldCorners(cameraPose);
            // project 3d target points into 2d image points
            var targetCorners = OpenCVHelp.projectPoints(
                cameraPose,
                prop,
                fieldCorners
            );
            // estimate pixel noise
            targetCorners = prop.estPixelNoise(targetCorners);
            // find the 2d yaw/pitch
            var boundingRect = OpenCVHelp.getBoundingRect(targetCorners);
            var yaw = prop.getPixelYaw(boundingRect.x);
            var pitch = prop.getPixelPitch(boundingRect.y);
            // find contour area
            double areaPixels = OpenCVHelp.getContourAreaPx(targetCorners);
            double areaPercent = areaPixels / prop.getResArea() * 100;

            // projected target can't be detected, skip to next
            if(!canSeeCorners(areaPixels, targetCorners)) continue;

            // only do 3d estimation if we have a planar target
            var pnpSim = new OpenCVHelp.PNPResults(new Transform3d(), new Transform3d(), 0);
            if(tgt.getModel().isPlanar) {
                pnpSim = OpenCVHelp.solvePNP(prop, tgt.getModel().cornerOffsets, targetCorners);
            }

            visibleTgts.add(
                new PhotonTrackedTarget(
                    yaw.getDegrees(),
                    pitch.getDegrees(),
                    areaPercent,
                    0.0,
                    tgt.id,
                    pnpSim.best,
                    pnpSim.alt,
                    pnpSim.ambiguity,
                    List.of(targetCorners)
                )
            );
        }

        // put this simulated data to NT
        submitProcessedFrame(latencyMillis, PhotonTargetSortMode.Largest, visibleTgts);
        return visibleTgts;
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param sortMode Order in which to sort targets
     * @param targetList List of targets detected
     */
    public void submitProcessedFrame(
            double latencyMillis, PhotonTargetSortMode sortMode, List<PhotonTrackedTarget> targetList) {
        if (sortMode != null) {
            targetList.sort(sortMode.getComparator());
        }
        submitProcessedFrame(latencyMillis, targetList);
    }
    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param targetList List of targets detected
     */
    public void submitProcessedFrame(double latencyMillis, List<PhotonTrackedTarget> targetList) {
        latencyMillisEntry.setDouble(latencyMillis);
        PhotonPipelineResult newResult = new PhotonPipelineResult(latencyMillis, targetList);
        var newPacket = new Packet(newResult.getPacketSize());
        newResult.populatePacket(newPacket);
        cam.rawBytesEntry.setRaw(newPacket.getData());

        boolean hasTargets = newResult.hasTargets();
        hasTargetEntry.setBoolean(hasTargets);
        if (!hasTargets) {
            targetPitchEntry.setDouble(0.0);
            targetYawEntry.setDouble(0.0);
            targetAreaEntry.setDouble(0.0);
            targetPoseEntry.setDoubleArray(new double[] {0.0, 0.0, 0.0});
            targetSkewEntry.setDouble(0.0);
        } else {
            var bestTarget = newResult.getBestTarget();

            targetPitchEntry.setDouble(bestTarget.getPitch());
            targetYawEntry.setDouble(bestTarget.getYaw());
            targetAreaEntry.setDouble(bestTarget.getArea());
            targetSkewEntry.setDouble(bestTarget.getSkew());

            var transform = bestTarget.getBestCameraToTarget();
            double[] poseData = {
                transform.getX(), transform.getY(), transform.getRotation().toRotation2d().getDegrees()
            };
            targetPoseEntry.setDoubleArray(poseData);
        }
    }
}
