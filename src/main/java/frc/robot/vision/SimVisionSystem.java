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
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import org.photonvision.targeting.TargetCorner;

public class SimVisionSystem {

    private final Map<String, PhotonCameraSim> camSimMap = new HashMap<>();

    private final TimeInterpolatableBuffer<Pose3d> robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);

    private Map<String, Set<SimVisionTarget>> tgtList = new HashMap<>();

    private final Field2d dbgField;

    /**
     * Create a simulated vision system involving a camera(s) and coprocessor(s) mounted on a mobile robot
     * running PhotonVision, detecting one or more targets scattered around the field.
     *
     * @param cameraSims The simulated cameras to process with
     */
    public SimVisionSystem(String visionSystemName, PhotonCameraSim... cameraSims) {
        dbgField = new Field2d();
        String tableName = "vision-"+visionSystemName;
        SmartDashboard.putData(tableName + "/Sim Field", dbgField);

        for(var cameraSim : cameraSims) {
            var existing = camSimMap.putIfAbsent(cameraSim.getCamera().name, cameraSim);
            if(existing == null) {
                cameraSim.logDebugCorners(tableName);
            }
        }
    }
    
    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of the camera
     * relative to the subset of these targets which are visible from the given camera position.
     * 
     * <p>By default these are added under the type "targets".
     *
     * @param targets Targets to add to the simulated field
     */
    public void addVisionTargets(SimVisionTarget... targets) {
        addVisionTargets("targets", targets);
    }
    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of the camera
     * relative to the subset of these targets which are visible from the given camera position.
     * 
     * <p>The AprilTags from this layout will be added as vision targets under the type "apriltags".
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
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of the camera
     * relative to the subset of these targets which are visible from the given camera position.
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
     * Get one of the simulated cameras.
     */
    public PhotonCameraSim getCameraSim(String name) {
        return camSimMap.get(name);
    }

    /**
     * Get the robot pose in meters saved by the vision system secondsAgo.
     * @param secondsAgo Seconds to look into the past
     */
    public Pose3d getRobotPose(double secondsAgo) {
        return robotPoseBuffer.getSample(Timer.getFPGATimestamp() - secondsAgo).orElse(new Pose3d());
    }

    public Field2d getDebugField() {
        return dbgField;
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

        // save "real" robot poses over time
        double now = Timer.getFPGATimestamp();
        robotPoseBuffer.addSample(now, robotPoseMeters);
        dbgField.setRobotPose(robotPoseMeters.toPose2d());

        var allTargets = new ArrayList<SimVisionTarget>();
        targetTypes.forEach((entry) -> allTargets.addAll(entry.getValue()));
        var visibleTargets = new ArrayList<Pose2d>();
        var cameraPose2ds = new ArrayList<Pose2d>();
        // process each camera
        for(var camSim : camSimMap.values()) {
            // check if this camera is ready to process and get latency
            var optionalLatency = camSim.getShouldProcess();
            if(optionalLatency.isEmpty()) continue;
            double latencyMillis = optionalLatency.get();
            // save "real" camera poses over time
            camSim.updateCameraPose(robotPoseMeters);
            // display camera latency milliseconds ago
            Pose3d cameraPose = camSim.getCameraPose(latencyMillis / 1000.0);
            cameraPose2ds.add(cameraPose.toPose2d());

            // update camera's visible targets
            var trackedTargets = camSim.process(latencyMillis, cameraPose, allTargets);
            var visibleCorners = new ArrayList<TargetCorner>();
            var bestCorners = new ArrayList<TargetCorner>();
            // display results
            for(var target : trackedTargets) {
                visibleTargets.add(
                    cameraPose.transformBy(target.getBestCameraToTarget()).toPose2d()
                );
                visibleCorners.addAll(target.getCorners());
                if(bestCorners.size()==0) bestCorners.addAll(target.getCorners());
            }
            var dbgCorners = camSim.getDebugCorners();
            dbgCorners.getObject("corners").setPoses(
                List.of(camSim.prop.getPixelFraction(visibleCorners.toArray(new TargetCorner[0])))
                    .stream()
                    .map(p -> new Pose2d(p.x, 1-p.y, new Rotation2d()))
                    .collect(Collectors.toList())
            );
            dbgCorners.getObject("bestCorners").setPoses(
                List.of(camSim.prop.getPixelFraction(bestCorners.toArray(new TargetCorner[0])))
                    .stream()
                    .map(p -> new Pose2d(p.x, 1-p.y, new Rotation2d()))
                    .collect(Collectors.toList())
            );
            dbgCorners.getObject("aspectRatio").setPoses(
                List.of(camSim.prop.getPixelFraction(
                    new TargetCorner(0, 0),
                    new TargetCorner(camSim.prop.getResWidth(), 0),
                    new TargetCorner(camSim.prop.getResWidth(), camSim.prop.getResHeight()),
                    new TargetCorner(0, camSim.prop.getResHeight())
                )).stream()
                    .map(p -> new Pose2d(p.x, 1-p.y, new Rotation2d()))
                    .collect(Collectors.toList())
            );
        }
        if(visibleTargets.size() != 0) dbgField.getObject("visibleTargets").setPoses(visibleTargets);
        if(cameraPose2ds.size() != 0) dbgField.getObject("cameras").setPoses(cameraPose2ds);
    }
}
