// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

/**
 * Custom implementation of
 * {@link edu.wpi.first.wpilibj2.command.SwerveControllerCommand SwerveControllerCommand}
 * to simplify construction, allow PathPlanner paths, and enable custom logging
 */
public class OCSwerveFollower extends CommandBase {

    private final SwerveDrive drivetrain;
    private final Trajectory trajectory;
    private Timer timer = new Timer();

    public OCSwerveFollower(SwerveDrive drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        addRequirements(drivetrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.resetPathController(); // reset theta setpoint between different trajectories
        drivetrain.logTrajectory(trajectory); // display the trajectory on field2d
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentTime = timer.get();
        // The target state of the trajectory right now (the robot's pose and velocity)
        Trajectory.State targetState = trajectory.sample(currentTime);
        Rotation2d targetRotation = targetState.poseMeters.getRotation();
        // Check if we are using PathPlanner trajectories
        if(trajectory instanceof PathPlannerTrajectory){
            targetState = ((PathPlannerTrajectory)trajectory).sample(currentTime);
            targetRotation = ((PathPlannerState)targetState).holonomicRotation;
        }

        drivetrain.drive(targetState, targetRotation);

        //Pose2d targetPose = targetState.poseMeters;
        //SmartDashboard.putNumber("Target Heading", targetPose.getRotation().getDegrees());
        //SmartDashboard.putNumber("Target X", targetPose.getX());
        //SmartDashboard.putNumber("Target Y", targetPose.getY());
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // the path is time parametrized and takes a certain number of seconds
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
