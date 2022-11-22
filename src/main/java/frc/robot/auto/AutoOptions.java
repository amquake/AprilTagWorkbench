package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AutoOptions {
    
    // list of choosable commands that decides what is run in auto
    private SendableChooser<Command> autoOptions = new SendableChooser<>();

    public AutoOptions(SwerveDrive drivetrain){

        autoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.stop(), drivetrain)
        );

        autoOptions.addOption("QuintupleRight",
            sequence(
                autoFollowTrajectory(
                    drivetrain, 
                    "path1", 
                    AutoConstants.kMediumSpeedConfig,
                    true
                ),
                drivetrain.stopC(),
                new WaitCommand(1),
                autoFollowTrajectory(
                    drivetrain, 
                    "path2", 
                    AutoConstants.kMediumSpeedConfig,
                    false
                ),
                drivetrain.stopC(),
                new WaitCommand(1),
                autoFollowTrajectory(
                    drivetrain, 
                    "path3", 
                    AutoConstants.kFastSpeedConfig,
                    false
                )
            )
        );        
    }

    /**
     * @param trajectories Any number of trajectories to perform in sequence
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowTrajectory(SwerveDrive drivetrain, Trajectory trajectory, boolean firstTrajectory){
        final Pose2d initial = (trajectory instanceof PathPlannerTrajectory) ?
            new Pose2d(
                trajectory.getInitialPose().getTranslation(),
                ((PathPlannerState)((PathPlannerTrajectory)trajectory).sample(0)).holonomicRotation)
            :
            trajectory.getInitialPose();
        Command followCommand = new OCSwerveFollower(drivetrain, trajectory);
        if(firstTrajectory){
            followCommand = followCommand.beforeStarting(()->drivetrain.resetOdometry(initial));
        }   
        return followCommand;
    }
    /**
     * @param config The config for this trajectory defining max velocity and acceleration
     * @param storedPathNames The names of the PathPlanner paths saved to this project for use in this trajectory
     * @return A command suitable for following a sequence of trajectories in autonomous.
     * The robot pose is reset to the start of the trajectory and {@link OCSwerveFollower} is used to follow it.
     */
    private Command autoFollowTrajectory(SwerveDrive drivetrain, String storedPathName, TrajectoryConfig config, boolean firstTrajectory){
        Trajectory trajectory = PathPlanner.loadPath(storedPathName, config.getMaxVelocity(), config.getMaxAcceleration(), config.isReversed());
        return autoFollowTrajectory(drivetrain, trajectory, firstTrajectory);
    }

    // Network Tables
    public Command getSelected(){
        return autoOptions.getSelected();
    }

    public void submit(){
        SmartDashboard.putData("Auto Options", autoOptions);
    }
}
