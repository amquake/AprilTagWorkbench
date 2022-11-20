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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonVersion;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
    
    /**
     * Constructs a handle for simulating PhotonCamera values.
     * Targets submitted through this class will change the simulated camera's results.
     * 
     * <p><b>This constructor's camera has a 90 deg FOV with no simulated lag!
     *
     * @param camera The camera to be simulated
     */
    public PhotonCameraSim(PhotonCamera camera) {
        this(camera, SimCamProperties.PERFECT_90DEG);
    }
    /**
     * Constructs a handle for simulating PhotonCamera values.
     * Targets submitted through this class will change the simulated camera's results.
     *
     * @param camera The camera to be simulated
     * @param prop Properties of this camera such as FOV and FPS
     */
    public PhotonCameraSim(PhotonCamera camera, SimCamProperties prop) {
        this.cam = camera;
        this.prop = prop;
        
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

    public PhotonCamera getCamera() {
        return cam;
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param targets Each target detected
     */
    public void submitProcessedFrame(double latencyMillis, PhotonTrackedTarget... targets) {
        submitProcessedFrame(latencyMillis, Arrays.asList(targets));
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param sortMode Order in which to sort targets
     * @param targets Each target detected
     */
    public void submitProcessedFrame(
            double latencyMillis, PhotonTargetSortMode sortMode, PhotonTrackedTarget... targets) {
        submitProcessedFrame(latencyMillis, sortMode, Arrays.asList(targets));
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param targetList List of targets detected
     */
    public void submitProcessedFrame(double latencyMillis, List<PhotonTrackedTarget> targetList) {
        submitProcessedFrame(latencyMillis, null, targetList);
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
        latencyMillisEntry.setDouble(latencyMillis);
        targetList = new ArrayList<>(targetList);
        if (sortMode != null) {
            targetList.sort(sortMode.getComparator());
        }

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
