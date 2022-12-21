package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.vision.estimation.CameraProperties;
import frc.robot.vision.estimation.VisionEstimation;
import frc.robot.vision.sim.PhotonCamera;
import frc.robot.vision.sim.PhotonCameraSim;
import frc.robot.vision.sim.SimVisionSystem;

public class RobotContainer {
    private final SwerveDrive drivetrain = new SwerveDrive();

    private OCXboxController controller = new OCXboxController(0);

    private final AutoOptions autoOptions = new AutoOptions(drivetrain);

    private final Field2d field;
    private AprilTagFieldLayout tagLayout;

    private NetworkTableInstance instance;
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final List<PhotonCamera> cameras;
    private List<PhotonPipelineResult> lastResults;
    private final SimVisionSystem visionSim;

    private boolean correcting = false;
    
    public RobotContainer() {
        autoOptions.submit();
        
        instance = NetworkTableInstance.getDefault();

        camera1 = new PhotonCamera(instance, "front");
        camera2 = new PhotonCamera(instance, "back");
        cameras = List.of(camera1, camera2);
        lastResults = new ArrayList<>(cameras.size());
        cameras.forEach(c -> lastResults.add(new PhotonPipelineResult()));

        visionSim = new SimVisionSystem("main");
        visionSim.addCamera(
            new PhotonCameraSim(
                camera1,
                CameraProperties.PI4_PICAM2_480p
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    Units.inchesToMeters(10),
                    0,
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    -Math.toRadians(18),
                    0
                )
            )
        );
        // visionSim.addCamera(
        //     new PhotonCameraSim(
        //         camera2,
        //         SimCamProperties.PI4_PICAM2_480p
        //     ),
        //     new Transform3d( // robot to camera
        //         new Translation3d(
        //             Units.inchesToMeters(-10),
        //             0,
        //             Units.inchesToMeters(25)
        //         ),
        //         new Rotation3d(
        //             0,
        //             -Math.toRadians(18),
        //             Math.PI
        //         )
        //     )
        // );            

        try {
            tagLayout = new AprilTagFieldLayout("2022-taglayout.json");
        } catch (IOException e){
            e.printStackTrace();
        };

        visionSim.addVisionTargets(tagLayout);
        field = visionSim.getDebugField();
    }

    public Command getAutoCommand() {
        return autoOptions.getSelected();
    }

    public void disable() {
        drivetrain.stop();
    }
    public void setAllBrake(boolean is) {
        drivetrain.setBrakeOn(is);
    }

    public void init() {
        controller = new OCXboxController(0);
        configureDriverBinds(controller);
    }

    public void configureDriverBinds(OCXboxController controller) {
        drivetrain.setDefaultCommand(
            new RunCommand(()->{
                drivetrain.drive(
                    controller.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                    controller.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                    controller.getTurn() * drivetrain.getMaxAngularVelocityRadians(),
                    true
                );
            }, drivetrain)
            .beforeStarting(()->controller.resetLimiters())
        );

        // push-to-change driving "speed"
        controller.rightBumper()
            .onTrue(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedMax)))
            .onFalse(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault)));

        controller.y()
        .whileTrue(run(()->{
            var cameraSim = visionSim.getCameraSim(camera1.name);
            visionSim.adjustCamera(cameraSim,
                new Transform3d(new Translation3d(), new Rotation3d(0, -0.01, 0))
                        .plus(visionSim.getRobotToCamera(cameraSim))
            );
        }));
        controller.a()
            .whileTrue(run(()->{
                var cameraSim = visionSim.getCameraSim(camera1.name);
                visionSim.adjustCamera(cameraSim,
                new Transform3d(new Translation3d(), new Rotation3d(0, 0.01, 0))
                        .plus(visionSim.getRobotToCamera(cameraSim))
            );
            }));
        
        controller.rightTrigger(0.2)
            .onTrue(runOnce(()-> correcting = true))
            .onFalse(runOnce(() -> correcting = false));

        controller.leftTrigger(0.2)
            .onTrue(runOnce(()->{
                var noise = new Transform2d(
                    new Translation2d(Math.random()*2-1, Math.random()*2-1),
                    new Rotation2d(Math.random()*4-2)
                );
                drivetrain.resetNoisyOdometry(drivetrain.getPose().plus(noise));
            }));
    }

    public void log() {
        drivetrain.log();

        field.getObject("Swerve Modules").setPoses(drivetrain.getModulePoses());
        
        Trajectory logTrajectory = drivetrain.getLogTrajectory();
        if(logTrajectory == null) logTrajectory = new Trajectory();
        field.getObject("Trajectory").setTrajectory(logTrajectory);
    }

    //----- Simulation
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPerfPose());
        field.getObject("Noisy Robot").setPose(drivetrain.getPerfPose());

        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();
        var relVisTagsPnP = new ArrayList<AprilTag>();
        var relVisTagsTrig = new ArrayList<AprilTag>();

        var bestPoses = new ArrayList<Pose2d>();
        var altPoses = new ArrayList<Pose2d>();
        var testPoses = new ArrayList<Pose2d>();

        boolean updated = false;
        for(int i = 0; i < cameras.size(); i++) {
            var camera = cameras.get(i);
            var cameraSim = visionSim.getCameraSim(camera.name);
            var robotToCamera = visionSim.getRobotToCamera(cameraSim);
            var result = camera.getLatestResult();

            if(result.equals(lastResults.get(i))) continue;
            else {
                lastResults.set(i, result);
                updated = true;
            }

            for(var target : result.getTargets()) {
                visCorners.addAll(target.getCorners());
                Pose3d tagPose = tagLayout.getTagPose(target.getFiducialId()).get();
                // actual layout poses of visible tags
                knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
                Transform3d camToBest = target.getBestCameraToTarget();
                // tags estimated relative to robot
                relVisTagsPnP.add(new AprilTag(
                    target.getFiducialId(),
                    new Pose3d().plus(robotToCamera).plus(camToBest)
                ));
                var undistortedTarget = cameraSim.prop.undistort2dTarget(target);
                relVisTagsTrig.addAll(VisionEstimation.estimateTagsTrig(
                    robotToCamera,
                    List.of(undistortedTarget),
                    tagLayout
                ));
                Transform3d camToAlt = target.getAlternateCameraToTarget();

                var bestPose = tagPose
                    .transformBy(camToBest.inverse())
                    .transformBy(robotToCamera.inverse())
                    .toPose2d();
                
                if(correcting) drivetrain.addVisionMeasurement(bestPose, result.getLatencyMillis()/1000.0);

                bestPoses.add(bestPose);
                altPoses.add(
                    tagPose
                        .transformBy(camToAlt.inverse())
                        .transformBy(robotToCamera.inverse())
                        .toPose2d()
                );
                testPoses.add(
                    new Pose3d(drivetrain.getPerfPose())
                        .plus(new Transform3d(new Pose3d(), relVisTagsTrig.get(relVisTagsTrig.size()-1).pose))
                        .toPose2d()
                );
            }

            // multi-target solvePNP
            if(result.getTargets().size() > 1) {
                var pnpResults = VisionEstimation.estimateCamPosePNP(
                    cameraSim.prop,
                    visCorners,
                    knownVisTags
                );
                var best = new Pose3d()
                    .plus(pnpResults.best) // field-to-camera
                    .plus(robotToCamera.inverse()); // field-to-robot
                var alt = new Pose3d()
                    .plus(pnpResults.alt) // field-to-camera
                    .plus(robotToCamera.inverse()); // field-to-robot
                bestPoses.clear();
                altPoses.clear();
                bestPoses.add(best.toPose2d());
                altPoses.add(alt.toPose2d());
                // testPoses.add(best.toPose2d());
            }
        }
        // multi-target SVD
        if(knownVisTags.size() > 0) {
            // var estTrf = VisionEstimation.estimateTransformLS(
            //     relVisTagsPnP,
            //     knownVisTags,
            //     true
            // );
            var estTrf = VisionEstimation.estimateTransformLS(
                relVisTagsTrig,
                knownVisTags,
                false
            );
            var estRobotPose = estTrf.trf.apply(new Pose3d());
            testPoses.add(estRobotPose.toPose2d());
        }
        if(updated) {
            field.getObject("bestPoses").setPoses(bestPoses);
            field.getObject("altPoses").setPoses(altPoses);
            field.getObject("testPoses").setPoses(testPoses);
        }
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        return sum;
    }
}
