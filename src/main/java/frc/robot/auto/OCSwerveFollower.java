// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class OCSwerveFollower extends CommandBase {
    
    private final SwerveDrive drivetrain;
    private final String pathName;
    private final PathConstraints constraints;
    private final boolean resetOdom;

    private CommandBase controllerCommand = Commands.none();

    public OCSwerveFollower(
            SwerveDrive drivetrain, String pathName,
            PathConstraints constraints, boolean resetOdom) {
        this.drivetrain = drivetrain;
        this.pathName = pathName;
        this.constraints = constraints;
        this.resetOdom = resetOdom;

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        var path = PathPlanner.loadPath(pathName, constraints);
        if(path == null) {
            end(false);
            return;
        }
        var alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(
            path,
            DriverStation.getAlliance()
        );

        if(resetOdom) drivetrain.resetOdometry(alliancePath.getInitialHolonomicPose());
        drivetrain.logTrajectory(alliancePath);

        controllerCommand = new PPSwerveControllerCommand(
            alliancePath,
            drivetrain::getPose,
            drivetrain.getXController(),
            drivetrain.getYController(),
            drivetrain.getRotController(),
            (chassisSpeeds)->drivetrain.setChassisSpeeds(chassisSpeeds, false, true),
            false,
            drivetrain
        );
        controllerCommand.initialize();
    }
    
    @Override
    public void execute() {
        controllerCommand.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        controllerCommand.end(interrupted);
        drivetrain.logTrajectory(null);
    }
    
    @Override
    public boolean isFinished() {
        return controllerCommand.isFinished();
    }
}
