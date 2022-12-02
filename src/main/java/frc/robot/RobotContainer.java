package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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
import frc.robot.vision.AprilTagFieldLayout;
import frc.robot.vision.SimCamProperties;
import frc.robot.vision.PhotonCamera;
import frc.robot.vision.PhotonCameraSim;
import frc.robot.vision.SimVisionSystem;
import frc.robot.vision.SimVisionTarget;

public class RobotContainer {
    private final SwerveDrive drivetrain = new SwerveDrive();

    private OCXboxController controller = new OCXboxController(0);

    private final AutoOptions autoOptions = new AutoOptions(drivetrain);

    private final Field2d field;
    private AprilTagFieldLayout tagLayout;

    private NetworkTableInstance instance;
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final SimVisionSystem visionSim;

    private boolean correcting = false;
    
    public RobotContainer() {
        autoOptions.submit();
        
        instance = NetworkTableInstance.getDefault();

        camera1 = new PhotonCamera(instance, "front");
        camera2 = new PhotonCamera(instance, "back");

        visionSim = new SimVisionSystem("main",
            new PhotonCameraSim(
                camera1,
                SimCamProperties.PI4_PICAM2_480p,
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
            )/*,
            new PhotonCameraSim(
                camera2,
                SimCamProperties.PI4_PICAM2_480p,
                new Transform3d( // robot to camera
                    new Translation3d(
                        Units.inchesToMeters(-10),
                        0,
                        Units.inchesToMeters(25)
                    ),
                    new Rotation3d(
                        0,
                        Math.toRadians(10),
                        Math.PI
                    )
                )
            )
            */
        );

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
        controller.rightBumper
            .onTrue(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedMax)))
            .onFalse(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault)));

        controller.yButton
        .whileTrue(run(()->{
            var cameraSim = visionSim.getCameraSim(camera1.name);
            cameraSim.adjustCamera(
                cameraSim.getRobotToCamera().plus(
                    new Transform3d(new Translation3d(), new Rotation3d(0, -0.01, 0))
                )
            );
        }));
        controller.aButton
            .whileTrue(run(()->{
                var cameraSim = visionSim.getCameraSim(camera1.name);
                cameraSim.adjustCamera(
                    cameraSim.getRobotToCamera().plus(
                        new Transform3d(new Translation3d(), new Rotation3d(0, 0.01, 0))
                    )
                );
            }));
        
        controller.rightTriggerButton
            .onTrue(runOnce(()-> correcting = true))
            .onFalse(runOnce(() -> correcting = false));

        controller.leftTriggerButton
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
        field.getObject("Noisy Robot").setPose(drivetrain.getPose());

        var bestPoses = new ArrayList<Pose2d>();
        var altPoses = new ArrayList<Pose2d>();

        for(var camera : List.of(camera1, camera2)) {
            var cameraSim = visionSim.getCameraSim(camera.name);
            var result = camera.getLatestResult();
            for(var target : result.getTargets()) {
                Pose3d tagPose = tagLayout.getTagPose(target.getFiducialId()).get();
                Transform3d camToBest = target.getBestCameraToTarget();
                Transform3d camToAlt = target.getAlternateCameraToTarget();

                var bestPose = tagPose
                    .transformBy(camToBest.inverse())
                    .transformBy(cameraSim.getRobotToCamera().inverse())
                    .toPose2d();
                
                if(correcting) drivetrain.addVisionMeasurement(bestPose, result.getLatencyMillis()/1000.0);

                bestPoses.add(bestPose);
                altPoses.add(
                    tagPose
                        .transformBy(camToAlt.inverse())
                        .transformBy(cameraSim.getRobotToCamera().inverse())
                        .toPose2d()
                );
            }
        }
        field.getObject("bestPoses").setPoses(bestPoses);
        field.getObject("altPoses").setPoses(altPoses);
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        return sum;
    }
}
