package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.vision.AprilTag;
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

    private final Field2d field = new Field2d();
    private final Field2d dbgCorners = new Field2d();
    private AprilTagFieldLayout tagLayout;

    private NetworkTableInstance instance;
    private final PhotonCamera camera;
    private final String cameraName = "picam";
    private final SimVisionSystem visionSim;
    
    public RobotContainer() {
        autoOptions.submit();
        
        instance = NetworkTableInstance.getDefault();

        camera = new PhotonCamera(instance, cameraName);

        visionSim = new SimVisionSystem(
            new PhotonCameraSim(
                camera,
                SimCamProperties.PI4_PICAM2_480p
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    0,
                    0,
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    Math.toRadians(10),
                    0
                )
            ),
            0.032 // min target area percent
        );

        try {
            tagLayout = new AprilTagFieldLayout("2022-taglayout.json");
        } catch (IOException e){
            e.printStackTrace();
        };

        visionSim.addVisionTargets(tagLayout);
        
        SmartDashboard.putData(field);
        field.getObject("tags").setPoses(
            tagLayout.getTags().stream().map(t->t.pose.toPose2d()).collect(Collectors.toList())
        );

        SmartDashboard.putData("vision sim corners", dbgCorners);
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
    }

    public void log() {
        drivetrain.log();

        field.setRobotPose(drivetrain.getPose());
        field.getObject("Swerve Modules").setPoses(drivetrain.getModulePoses());
        
        Trajectory logTrajectory = drivetrain.getLogTrajectory();
        if(logTrajectory == null) logTrajectory = new Trajectory();
        field.getObject("Trajectory").setTrajectory(logTrajectory);
    }

    //----- Simulation
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
        List<Pose2d> bestPoses = new ArrayList<>();
        List<Pose2d> altPoses = new ArrayList<>();
        PhotonPipelineResult result = camera.getLatestResult();
        List<Pose2d> visibleTargets = new ArrayList<>();
        List<TargetCorner> visibleCorners = new ArrayList<>();

        if(result.hasTargets()) {
            var targets = result.getTargets();

            for(var target : targets) {
                Pose3d tagPose = tagLayout.getTagPose(target.getFiducialId()).get();
                Transform3d camToBest = target.getBestCameraToTarget();
                Transform3d camToAlt = target.getAlternateCameraToTarget();

                bestPoses.add(tagPose.transformBy(camToBest.inverse()).toPose2d());
                altPoses.add(tagPose.transformBy(camToAlt.inverse()).toPose2d());
                // visualize detected targets
                visibleTargets.add(
                    visionSim.getCameraPose(result.getLatencyMillis() / 1000.0)
                        .transformBy(camToBest).toPose2d()
                );
                visibleCorners.addAll(target.getCorners());
            }
        }
        field.getObject("bestPoses").setPoses(bestPoses);
        field.getObject("altPoses").setPoses(altPoses);

        // display visible targets and corners
        field.getObject("visibleTargets").setPoses(visibleTargets);

        var cornerPoses = new ArrayList<Pose2d>();
        var prop = visionSim.getCameraSim().prop;

        double aspectRatio = (double)prop.getResWidth() / prop.getResHeight();
        boolean wide = aspectRatio > 1;
        double widthFill = wide ? 0 : (1 - aspectRatio);
        double heightFill = wide ? (1 - 1/aspectRatio) : 0;

        for(TargetCorner corner : visibleCorners) {
            // offset to account for aspect ratio
            double x = corner.x / prop.getResWidth();
            double y = 1 - corner.y / prop.getResHeight();
            double offsetX = widthFill/2 + x - widthFill*x;
            double offsetY = heightFill/2 + y - heightFill*y;
            cornerPoses.add(new Pose2d(
                new Translation2d(
                    offsetX,
                    offsetY
                ),
                new Rotation2d()
            ));
        }
        dbgCorners.getObject("corners").setPoses(cornerPoses);
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        return sum;
    }
}
