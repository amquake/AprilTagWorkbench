package frc.robot.vision;

import java.util.Random;

import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;

    /**
     * Calibration and performance values for this camera.
     * 
     * <p>The resolution will affect the accuracy of projected(3d->2d) target corners and
     * similarly the severity of image noise on estimation(2d->3d).
     * 
     * <p>The camera intrinsics and distortion coefficients describe the results of calibration,
     * and how to map between 3d field points and 2d image points.
     * 
     * <p>The performance values (frame/shutter speed, latency) determine how often results
     * are updated and with how much latency. Low shutter speeds cause motion blur which
     * can inhibit target detection while moving. Note that latency estimation does not
     * account for network latency and the latency reported in the PhotonCamera's results
     * will always be perfect.
     */
    public class SimCamProperties {
        private final Random rand = new Random();
        // calibration
        private int resWidth;
        private int resHeight;
        private Matrix<N3, N3> camIntrinsics;
        private Vector<N5> distCoeffs;
        private double avgErrorPx;
        private double errorStdDevPx;
        // performance
        private double frameSpeedMs = 0;
        private double shutterSpeedMs = 0;
        private double avgLatencyMs = 0;
        private double latencyStdDevMs = 0;

        /**
         * Default constructor which is the same as {@link #PERFECT_90DEG}
         */
        public SimCamProperties(){
            setCalibration(960, 720, Rotation2d.fromDegrees(90));
        };

        public void setCalibration(int resWidth, int resHeight, Rotation2d fovDiag) {
            var fovWidth = new Rotation2d(
                fovDiag.getRadians() * (resWidth / Math.sqrt(resWidth*resWidth + resHeight*resHeight))
            );
            var fovHeight = new Rotation2d(
                fovDiag.getRadians() * (resHeight / Math.sqrt(resWidth*resWidth + resHeight*resHeight))
            );
            // assume no distortion
            var distCoeff = VecBuilder.fill(0,0,0,0,0);
            // assume centered principal point (pixels)
            double cx = resWidth/2.0;
            double cy = resHeight/2.0;
            // use given fov to determine focal point (pixels)
            double fx = cx / Math.tan(fovWidth.getRadians()/2.0);
            double fy = cy / Math.tan(fovHeight.getRadians()/2.0);
            // create camera intrinsics matrix
            var camIntrinsics = Matrix.mat(Nat.N3(), Nat.N3()).fill(
                fx, 0, cx,
                0, fy, cy,
                0,  0, 1
            );
            setCalibration(resWidth, resHeight, camIntrinsics, distCoeff);
        }
        public void setCalibration(int resWidth, int resHeight,
                Matrix<N3, N3> camIntrinsics, Vector<N5> distCoeffs) {
            this.resWidth = resWidth;
            this.resHeight = resHeight;
            this.camIntrinsics = camIntrinsics;
            this.distCoeffs = distCoeffs;
        }
        public void setCalibError(double avgErrorPx, double errorStdDevPx) {
            this.avgErrorPx = avgErrorPx;
            this.errorStdDevPx = errorStdDevPx;
        }
        /**
         * @param fps The average frames per second the camera should process at
         */
        public void setFPS(double fps) {
            this.frameSpeedMs = 1000.0 / fps;
        }
        /**
         * @param shutterSpeedMs The amount of time the shutter is open for one frame.
         *     Affects motion blur. <b>Frame speed(from FPS) is limited to this!</b>
         */
        public void setShutterSpeedMs(double shutterSpeedMs) {
            this.shutterSpeedMs = shutterSpeedMs;
        }
        /**
         * @param avgLatencyMs The average latency (image capture -> data) in milliseconds
         *     a frame should have
         */
        public void setAvgLatencyMs(double avgLatencyMs) {
            this.avgLatencyMs = avgLatencyMs;
        }
        /**
         * @param latencyStdDevMs The standard deviation in milliseconds of the latency
         */
        public void setLatencyStdDevMs(double latencyStdDevMs) {
            this.latencyStdDevMs = latencyStdDevMs;
        }

        public int getResWidth() {
            return resWidth;
        }
        public int getResHeight() {
            return resHeight;
        }
        public int getResArea() {
            return resWidth*resHeight;
        }
        
        public Matrix<N3, N3> getIntrinsics() {
            return camIntrinsics.copy();
        }
        /**
         * The yaw from the principal point of this camera to the pixel x value.
         */
        public Rotation2d getPixelYaw(double pixelX) {
            double fx = camIntrinsics.get(0, 0);
            // account for principal point not being centered
            double cx = camIntrinsics.get(0, 2);
            double xOffset = cx - pixelX;
            return new Rotation2d(
                fx,
                xOffset
            );
        }
        /**
         * The pitch from the principal point of this camera to the pixel y value.
         */
        public Rotation2d getPixelPitch(double pixelY) {
            double fy = camIntrinsics.get(1, 1);
            // account for principal point not being centered
            double cy = camIntrinsics.get(1, 2);
            double yOffset = cy - pixelY;
            return new Rotation2d(
                fy,
                yOffset
            );
        }
        public Rotation2d getHorizFOV() {
            // sum of FOV left and right principal point
            var left = getPixelYaw(0);
            var right = getPixelYaw(resWidth);
            return left.minus(right);
        }
        public Rotation2d getVertFOV() {
            // sum of FOV above and below principal point
            var above = getPixelPitch(0);
            var below = getPixelPitch(resHeight);
            return above.minus(below);
        }
        public Rotation2d getDiagFOV() {
            return new Rotation2d(Math.hypot(getHorizFOV().getRadians(), getVertFOV().getRadians()));
        }
        /** Width:height */
        public double getAspectRatio() {
            return (double)resWidth / resHeight;
        }
        /**
         * Returns these pixel points as fractions of a 1x1 square image.
         * This means the camera's aspect ratio and resolution will be used, and the
         * points' x and y may not reach all portions(e.g. a wide aspect ratio means
         * some of the top and bottom of the square image is unreachable).
         * @param corners Pixel points on this camera's image
         * @return Points mapped to an image of 1x1 resolution
         */
        public TargetCorner[] getPixelFraction(TargetCorner... corners) {
            double resLarge = getAspectRatio() > 1 ? resWidth : resHeight;

            var newCorners = corners.clone();
            for(int i=0; i<corners.length; i++) {
                // offset to account for aspect ratio
                newCorners[i] = new TargetCorner(
                    (corners[i].x + (resLarge-resWidth)/2.0) / resLarge,
                    (corners[i].y + (resLarge-resHeight)/2.0) / resLarge
                );
            }
            return newCorners;
        }
        public Vector<N5> getDistCoeffs() {
            return new Vector<>(distCoeffs);
        }

        public double getFPS() {
            return 1000.0 / frameSpeedMs;
        }
        public double getFrameSpeedMs() {
            return frameSpeedMs;
        }
        public double getShutterSpeedMs() {
            return shutterSpeedMs;
        }
        public double getAvgLatencyMs() {
            return avgLatencyMs;
        }
        public double getLatencyStdDevMs() {
            return latencyStdDevMs;
        }

        /**
         * @return Estimate of new target corners based on this camera's noise.
         */
        public TargetCorner[] estPixelNoise(TargetCorner... corners) {
            if(avgErrorPx == 0 && errorStdDevPx == 0) return corners;

            var newCorners = corners.clone();
            for(int i=0; i<corners.length; i++) {
                var corn = corners[i];
                // error pixels in random direction
                double error = rand.nextGaussian(avgErrorPx, errorStdDevPx);
                double errorAngle = rand.nextDouble(-Math.PI, Math.PI);

                newCorners[i] = new TargetCorner(
                    corn.x + error*Math.cos(errorAngle),
                    corn.y + error*Math.sin(errorAngle)
                );
            }
            return newCorners;
        }

        /**
         * @return Noisy estimation of a frame's processing latency in milliseconds
         */
        public double estLatencyMs() {
            return Math.max(rand.nextGaussian(avgLatencyMs, latencyStdDevMs), 0);
        }
        /**
         * @return Estimate how long until the next frame should be processed in milliseconds
         */
        public double estMsUntilNextFrame() {
            // exceptional processing latency blocks the next frame
            return frameSpeedMs + Math.max(0, estLatencyMs() - frameSpeedMs);
        }

        // pre-calibrated example cameras

        /** 960x720 resolution, 90 degree FOV, "perfect" lagless camera */
        public static final SimCamProperties PERFECT_90DEG = new SimCamProperties();
        public static final SimCamProperties PI4_PICAM2_480p = new SimCamProperties();
        static {
            PI4_PICAM2_480p.setCalibration(640, 480,
                Matrix.mat(Nat.N3(), Nat.N3()).fill( // intrinsic
                    497.8204072694636,  0.0,                314.53659309737424,
                    0.0,                481.6955284883231,  231.95042993880858,
                    0.0,                0.0,                1.0
                ),
                VecBuilder.fill( // distort
                    0.16990717177326176,
                    -0.47305087536583684,
                    -0.002992219989630736,
                    -0.0016840919550094836,
                    0.36623021008942863
                )
            );
            PI4_PICAM2_480p.setCalibError(0.25, 0.07);
            PI4_PICAM2_480p.setFPS(17);
            PI4_PICAM2_480p.setAvgLatencyMs(35);
            PI4_PICAM2_480p.setLatencyStdDevMs(8);
        }
    }