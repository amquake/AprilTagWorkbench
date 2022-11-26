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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimVisionTarget {

    /**
     * Describes the shape of the target
     */
    public static class TargetModel {
        /**
         * Translations of this target's corners relative to its pose
         */
        public final Translation3d[] cornerOffsets;
        public final boolean isPlanar;
        public final double widthMeters;
        public final double heightMeters;
        public final double areaSqMeters;

        public TargetModel(Translation3d[] cornerOffsets, double widthMeters, double heightMeters) {
            if(cornerOffsets == null || cornerOffsets.length == 0) {
                cornerOffsets = new Translation3d[]{new Translation3d()};
                this.isPlanar = false;
            }
            else {
                boolean cornersPlanar = true;
                for(Translation3d corner : cornerOffsets) {
                    if(corner.getX() != 0) cornersPlanar = false;
                }
                if(cornerOffsets.length != 4 || !cornersPlanar) {
                    throw new IllegalArgumentException(
                        String.format(
                            "Supplied target corners (%s) must total 4 and be planar (all X == 0).",
                            cornerOffsets.length
                        )
                    );
                };
                this.isPlanar = true;
            }
            this.cornerOffsets = cornerOffsets;
            this.widthMeters = widthMeters;
            this.heightMeters = heightMeters;
            this.areaSqMeters = widthMeters * heightMeters;
        }
        /**
         * Creates a rectangular, planar target model given the width and height.
         */
        public static TargetModel ofPlanarRect(double widthMeters, double heightMeters) {
            // 4 corners of rect with its pose as origin
            return new TargetModel(
                new Translation3d[]{
                    // this order is relevant for solvePNP
                    new Translation3d(0, widthMeters/2.0, -heightMeters/2.0),
                    new Translation3d(0, -widthMeters/2.0, -heightMeters/2.0),
                    new Translation3d(0, -widthMeters/2.0, heightMeters/2.0),
                    new Translation3d(0, widthMeters/2.0, heightMeters/2.0)
                },
                widthMeters, heightMeters
            );
        }
        /**
         * Creates a spherical target which has similar dimensions when viewed from any angle.
         */
        public static TargetModel ofSphere(double diameterMeters) {
            // to get area = PI*r^2
            double assocSideLengths = Math.sqrt(Math.PI)*(diameterMeters/2.0);
            return new TargetModel(null, assocSideLengths, assocSideLengths);
        }

        @Override
        public boolean equals(Object obj) {
            if(this == obj) return true;
            if(obj instanceof TargetModel) {
                var o = (TargetModel)obj;
                return cornerOffsets.equals(o.cornerOffsets) &&
                        widthMeters == o.widthMeters &&
                        heightMeters == o.heightMeters &&
                        areaSqMeters == o.areaSqMeters;
            }
            return false;
        }
    }

    private Pose3d pose;
    private TargetModel model;
    /**
     * Translations of the target's corners on the field
     */
    private Translation3d[] fieldToCorners = new Translation3d[0];
    public final int id;

    /**
     * Describes a vision target located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the tag in field-relative coordinates
     * @param model TargetModel which describes the shape of the target
     */
    public SimVisionTarget(Pose3d pose, TargetModel model) {
        this.pose = pose;
        this.model = model;
        updateFieldCorners();
        this.id = -1;
    }
    /**
     * Describes an orientation-agnostic vision target (like a sphere) located somewhere on the field
     * that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the target in field-relative coordinates
     * @param diamMeters Size(diameter) of the outer bounding box of the target in meters.
     */
    public SimVisionTarget(Pose3d pose, double diamMeters) {
        this(pose, diamMeters, diamMeters, -1);
    }
    /**
     * Describes a planar vision target located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the target in field-relative coordinates
     * @param widthMeters Width of the outer bounding box of the target in meters.
     * @param heightMeters Height of the outer bounding box of the target in meters.
     */
    public SimVisionTarget(Pose3d pose, double widthMeters, double heightMeters) {
        this(pose, widthMeters, heightMeters, -1);
    }
    /**
     * Describes a fiducial tag located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the tag in field-relative coordinates
     * @param lengthMeters Width/height of the outer bounding box of the tag(black square) in meters.
     * @param id The ID of this fiducial tag
     */
    public SimVisionTarget(Pose3d pose, double lengthMeters, int id) {
        this(pose, lengthMeters, lengthMeters, id);
    }
    /**
     * Describes a fiducial tag located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param pose Pose3d of the tag in field-relative coordinates
     * @param widthMeters Width of the outer bounding box of the tag(black square) in meters.
     * @param heightMeters Height of the outer bounding box of the tag(black square) in meters.
     * @param id The ID of this fiducial tag
     */
    public SimVisionTarget(Pose3d pose, double widthMeters, double heightMeters, int id) {
        this.pose = pose;
        this.model = TargetModel.ofPlanarRect(widthMeters, heightMeters);
        updateFieldCorners();
        this.id = id;
    }

    public void setPose(Pose3d pose) {
        this.pose = pose;
        updateFieldCorners();
    }
    public void setModel(TargetModel model) {
        this.model = model;
        updateFieldCorners();
    }

    private void updateFieldCorners() {
        fieldToCorners = new Translation3d[model.cornerOffsets.length];
        for(int i=0; i<model.cornerOffsets.length; i++) {
            fieldToCorners[i] = pose.transformBy(
                new Transform3d(model.cornerOffsets[i], new Rotation3d())
            ).getTranslation();
        }
    }

    public Pose3d getPose() {
        return pose;
    }
    public TargetModel getModel(){ 
        return model;
    }
    /** This target's corners offset from its field pose. */
    public Translation3d[] getFieldCorners() {
        return fieldToCorners;
    }
    /** This target's corners offset from its field pose, which is facing the camera. */
    public Translation3d[] getAgnosticFieldCorners(Pose3d cameraPose) {
        var rel = new CameraTargetRelation(cameraPose, pose);
        // this target's pose but facing the camera pose
        var facingPose = new Pose3d(
            pose.getTranslation(),
            new Rotation3d(0, rel.camToTargPitch.getRadians(), rel.camToTargYaw.getRadians())
        );
        // find field corners based on this model's width/height if it was facing the camera
        return new SimVisionTarget(
            facingPose,
            TargetModel.ofPlanarRect(model.widthMeters, model.heightMeters)
        ).getFieldCorners();
    }

    @Override
    public boolean equals(Object obj) {
        if(this == obj) return true;
        if(obj instanceof SimVisionTarget) {
            var o = (SimVisionTarget)obj;
            return pose.equals(o.pose) &&
                    model.equals(o.model);
        }
        return false;
    }

    /** Holds various helper geometries describing the relation between camera and target. */
    public static class CameraTargetRelation {
        public final Pose3d camPose;
        public final Transform3d camToTarg;
        public final double camToTargDist;
        public final double camToTargDistXY;
        public final Rotation2d camToTargYaw;
        public final Rotation2d camToTargPitch;
        /** Angle from the camera's relative x-axis */
        public final Rotation2d camToTargAngle;
        public final Transform3d targToCam;
        public final Rotation2d targToCamYaw;
        public final Rotation2d targToCamPitch;
        /** Angle from the target's relative x-axis */
        public final Rotation2d targToCamAngle;
        public CameraTargetRelation(Pose3d cameraPose, Pose3d targetPose) {
            this.camPose = cameraPose;
            camToTarg = new Transform3d(cameraPose, targetPose);
            camToTargDist = camToTarg.getTranslation().getNorm();
            camToTargDistXY = Math.hypot(
                camToTarg.getTranslation().getX(),
                camToTarg.getTranslation().getY()
            );
            camToTargYaw = new Rotation2d(camToTarg.getX(), camToTarg.getY());
            camToTargPitch = new Rotation2d(camToTargDistXY, -camToTarg.getZ());
            camToTargAngle = new Rotation2d(Math.hypot(
                camToTargYaw.getRadians(),
                camToTargPitch.getRadians()
            ));
            targToCam = new Transform3d(targetPose, cameraPose);
            targToCamYaw = new Rotation2d(targToCam.getX(), targToCam.getY());
            targToCamPitch = new Rotation2d(camToTargDistXY, -targToCam.getZ());
            targToCamAngle = new Rotation2d(Math.hypot(
                targToCamYaw.getRadians(),
                targToCamPitch.getRadians()
            ));
        }
    }
}
