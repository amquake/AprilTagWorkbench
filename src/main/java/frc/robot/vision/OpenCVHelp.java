/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
package frc.robot.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.MatrixType;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.imgproc.Imgproc;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;

public final class OpenCVHelp {

    static {
        try {
            CameraServerCvJNI.forceLoad();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load native libraries!", e);
        }
    }

    public static MatOfDouble matrixToMat(SimpleMatrix matrix) {
        var mat = new Mat(matrix.numRows(), matrix.numCols(), CvType.CV_64F);
        mat.put(0, 0, matrix.getDDRM().getData());
        var wrappedMat = new MatOfDouble();
        mat.convertTo(wrappedMat, CvType.CV_64F);
        mat.release();
        return wrappedMat;
    }
    public static SimpleMatrix matToMatrix(Mat mat) {
        return new SimpleMatrix(mat.rows(), mat.cols(), true, mat.get(0, 0));
    }

    /**
     * Creates a new {@link MatOfPoint3f} with these 3d translations.
     * The opencv tvec is a vector with three elements representing {x, y, z} in the
     * EDN coordinate system.
     * @param tvecOutput The tvec to be filled with data
     */
    public static MatOfPoint3f translationToTvec(Translation3d... translations) {
        Point3[] points = new Point3[translations.length];
        for(int i = 0; i < translations.length; i++) {
            var trl = CoordinateSystem.convert(
                translations[i],
                CoordinateSystem.NWU(),
                CoordinateSystem.EDN()
            );
            points[i] = new Point3(trl.getX(), trl.getY(), trl.getZ());
        }
        return new MatOfPoint3f(points);
    }
    /**
     * Returns a new 3d translation from this {@link Mat}.
     * The opencv tvec is a vector with three elements representing {x, y, z} in the
     * EDN coordinate system.
     */
    public static Translation3d tvecToTranslation(Mat tvecInput) {
        double[] data = new double[3];
        tvecInput.get(0, 0, data);
        return CoordinateSystem.convert(
            new Translation3d(data[0], data[1], data[2]),
            CoordinateSystem.EDN(),
            CoordinateSystem.NWU()
        );
    }

    /**
     * Creates a new {@link MatOfPoint3f} with this 3d rotation.
     * The opencv rvec Mat is a vector with three elements representing the axis
     * scaled by the angle in the EDN coordinate system.
     * (angle = norm, and axis = rvec / norm)
     * @param rvecOutput The rvec Mat to be filled with data
     */
    public static MatOfPoint3f rotationToRvec(Rotation3d rotation) {
        rotation = CoordinateSystem.convert(
            rotation,
            CoordinateSystem.NWU(),
            CoordinateSystem.EDN()
        );
        var angle = rotation.getAngle();
        var axis = rotation.getAxis().times(angle);
        return new MatOfPoint3f(new Point3(axis.getData()));
    }
    /**
     * Returns a 3d rotation from this {@link Mat}.
     * The opencv rvec Mat is a vector with three elements representing the axis
     * scaled by the angle in the EDN coordinate system.
     * (angle = norm, and axis = rvec / norm)
     */
    public static Rotation3d rvecToRotation(Mat rvecInput) {
        double[] data = new double[3];
        rvecInput.get(0, 0, data);
        Vector<N3> axis = new Vector<>(
            Matrix.mat(Nat.N3(), Nat.N1())
                .fill(Arrays.copyOf(data, 3))
        );
        return CoordinateSystem.convert(
            new Rotation3d(axis, Core.norm(rvecInput)),
            CoordinateSystem.EDN(),
            CoordinateSystem.NWU()
        );
    }
    
    public static MatOfPoint2f targetCornersToMat(TargetCorner... corners) {
        var points = new Point[corners.length];
        for(int i=0; i<corners.length; i++) {
            points[i] = new Point(corners[i].x, corners[i].y);
        }
        return new MatOfPoint2f(points);
    }
    public static TargetCorner[] matToTargetCorners(MatOfPoint2f matInput) {
        var corners = new TargetCorner[(int)matInput.total()];
        float[] data = new float[(int)matInput.total()*matInput.channels()];
        matInput.get(0, 0, data);
        for(int i=0; i<corners.length; i++) {
            corners[i] = new TargetCorner(data[0+2*i], data[1+2*i]);
        }
        return corners;
    }

    public static TargetCorner[] projectPoints(
            Pose3d camPose, SimCamProperties camProp,
            Translation3d... objectTranslations) {
        // set object points relative to camera so we dont have to use rvec/tvec
        var relativeTrls = objectTranslations.clone();
        for(int i=0; i<objectTranslations.length; i++) {
            var trl = new Pose3d(
                objectTranslations[i],
                new Rotation3d()
            ).relativeTo(camPose).getTranslation();
            relativeTrls[i] = trl;
        }
        
        // translate to opencv classes
        var objectPoints = translationToTvec(relativeTrls);
        var rvec = Mat.zeros(3, 1, CvType.CV_32F);
        var tvec = Mat.zeros(3, 1, CvType.CV_32F);
        var cameraMatrix = matrixToMat(camProp.getIntrinsics().getStorage());
        var distCoeffs = matrixToMat(camProp.getDistCoeffs().getStorage());
        var imagePoints = new MatOfPoint2f();
        // project to 2d
        Calib3d.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
        
        /*
        System.err.println("--------------------------");
        System.err.println("rvec: "+rvec.dump());
        System.err.println("tvec: "+tvec.dump());
        System.err.println("cam: "+cameraMatrix.dump());
        System.err.println("dist: "+distortionCoeffs.dump());
        System.err.println("object: "+objectPoints.dump());
        System.err.println("image: "+imagePoints.dump());
        */
        
        // turn 2d point Mat into TargetCorners
        var corners = matToTargetCorners(imagePoints);

        // release our Mats from native memory
        objectPoints.release();
        rvec.release();
        tvec.release();
        cameraMatrix.release();
        distCoeffs.release();
        imagePoints.release();

        return corners;
    }
    public static double getPolygonArea(TargetCorner... corners) {
        var temp = targetCornersToMat(corners);
        var corn = new MatOfPoint(temp.toArray());
        temp.release();
        
        // outputHull gives us indices (of corn) that make a convex hull contour
        var outputHull = new MatOfInt();
        Imgproc.convexHull(corn, outputHull);
        int[] indices = outputHull.toArray();
        outputHull.release();
        var tempPoints = corn.toArray();
        var points = tempPoints.clone();
        for(int i=0; i<indices.length; i++) {
            points[i] = tempPoints[indices[i]];
        }
        corn.fromArray(points);
        // calculate area of the contour
        double area = Imgproc.contourArea(corn);
        corn.release();
        return area;
    }
    public static PNPResults solvePNP(
            SimCamProperties camProp, Translation3d[] modelTrls, TargetCorner[] imageCorners) {
        // translate to opencv classes
        var objectPoints = translationToTvec(modelTrls);
        var imagePoints = targetCornersToMat(imageCorners);
        var cameraMatrix = matrixToMat(camProp.getIntrinsics().getStorage());
        var distCoeffs = matrixToMat(camProp.getDistCoeffs().getStorage());
        var rvecs = new ArrayList<Mat>();
        var tvecs = new ArrayList<Mat>();
        var rvec = Mat.zeros(3, 1, CvType.CV_32F);
        var tvec = Mat.zeros(3, 1, CvType.CV_32F);
        var reprojectionError = new Mat();
        // calc rvecs/tvecs and associated reprojection error from image points
        Calib3d.solvePnPGeneric(
            objectPoints, imagePoints,
            cameraMatrix, distCoeffs,
            rvecs, tvecs,
            false, Calib3d.SOLVEPNP_IPPE_SQUARE,
            rvec, tvec,
            reprojectionError
        );

        float[] errors = new float[2];
        reprojectionError.get(0, 0, errors);
        double ambiguity = 0;
        // convert to wpilib coordinates
        var best = new Transform3d(
            tvecToTranslation(tvecs.get(0)),
            rvecToRotation(rvecs.get(0))
        );
        var alt = new Transform3d();

        if(tvecs.size() > 1) {
            alt = new Transform3d(
                tvecToTranslation(tvecs.get(1)),
                rvecToRotation(rvecs.get(1))
            );

            ambiguity = errors[0] / errors[1];
            if(errors[0] > errors[1]) {
                var temp = best;
                best = alt;
                alt = temp;
                ambiguity = errors[1] / errors[0];
            }
        }

        best = CoordinateSystem.convert(best, CoordinateSystem.NWU(), CoordinateSystem.EDN());
        best = convertOpenCVtoPhotonPose(best);
        alt = CoordinateSystem.convert(alt, CoordinateSystem.NWU(), CoordinateSystem.EDN());
        alt = convertOpenCVtoPhotonPose(alt);
        
        var results = new PNPResults(best, alt, ambiguity);

        /*
        System.err.println("--------------------------");
        System.err.println("object: "+objectPoints.dump());
        System.err.println("image: "+imagePoints.dump());
        for(int i=0; i<tvecs.size(); i++) {
            System.err.println("rvec["+i+"]: "+rvecs.get(i).dump());
            System.err.println("tvec["+i+"]: "+tvecs.get(i).dump());
        }
        System.err.println("reproj: "+reprojectionError.dump());
        */
        

        // release our Mats from native memory
        objectPoints.release();
        imagePoints.release();
        cameraMatrix.release();
        distCoeffs.release();
        for(var v : rvecs) v.release();
        for(var v : tvecs) v.release();
        rvec.release();
        tvec.release();
        reprojectionError.release();
        return results;
    }

    private static final Rotation3d wpilib = new Rotation3d(
        new MatBuilder<>(Nat.N3(), Nat.N3()).fill(0, 1, 0, 0, 0, 1, 1, 0, 0)
    ).plus(new Rotation3d(0, 0, Math.PI));
    private static Transform3d convertOpenCVtoPhotonPose(Transform3d camToTarg) {
        // CameraToTarget _should_ be in opencv-land EDN
        var nwu = CoordinateSystem.convert(
            new Pose3d(
                camToTarg.getTranslation(),
                camToTarg.getRotation()
            ),
            CoordinateSystem.EDN(),
            CoordinateSystem.NWU()
        );
        return new Transform3d(nwu.getTranslation(), wpilib.rotateBy(nwu.getRotation()));
    }

    public static class PNPResults {
        public final Transform3d best;
        public final Transform3d alt;
        public final double ambiguity;
        public PNPResults(Transform3d best, Transform3d alt, double ambiguity) {
            this.best = best;
            this.alt = alt;
            this.ambiguity = ambiguity;
        }
    }
}
