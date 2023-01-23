package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<Command> autoOptions = new SendableChooser<>();

    public AutoOptions(SwerveDrive drivetrain){

        autoOptions.setDefaultOption("Nothing",
            runOnce(()->drivetrain.stop(), drivetrain)
        );

        autoOptions.addOption("QuintupleRight",
            sequence(
                new OCSwerveFollower(
                    drivetrain, 
                    "path1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                drivetrain.stopC(),
                waitSeconds(1),
                new OCSwerveFollower(
                    drivetrain, 
                    "path2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                ),
                drivetrain.stopC(),
                waitSeconds(1),
                new OCSwerveFollower(
                    drivetrain, 
                    "path3", 
                    AutoConstants.kFastSpeedConfig,
                    false
                ),
                drivetrain.stopC()
            )
        );        
    }

    /**
     * Loads this path and then returns a command that follows it. The drivetrain will be
     * automatically stopped after the path ends.
     * 
     * @param drivetrain
     * @param pathName
     * @param constraints
     * @param resetOdom If the odometry should be reset to the first pose in this path
     * @return
     */
    public static CommandBase loadAndFollow(
            SwerveDrive drivetrain, String pathName,
            PathConstraints constraints, boolean resetOdom) {
        var path = PathPlanner.loadPath(pathName, constraints);
        if(path == null) return none();
        var alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(
            path,
            DriverStation.getAlliance()
        );

        return new PPSwerveControllerCommand(
            alliancePath,
            drivetrain::getPose,
            drivetrain.getXController(),
            drivetrain.getYController(),
            drivetrain.getRotController(),
            (chassisSpeeds)->drivetrain.setChassisSpeeds(chassisSpeeds, false, true),
            false,
            drivetrain
        ).beforeStarting(()->{
            if(resetOdom) drivetrain.resetOdometry(alliancePath.getInitialPose());
            drivetrain.logTrajectory(alliancePath);
        });
    }

    // Network Tables
    public Command getSelected(){
        return autoOptions.getSelected();
    }

    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}
