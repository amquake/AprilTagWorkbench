// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {

    private RobotContainer container;

    private Command autoCommand;

    @Override
    public void robotInit() {
        container = new RobotContainer();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        container.log();
    }
    
    @Override
    public void autonomousInit() {
        container.setAllBrake(true);

        autoCommand = container.getAutoCommand();

        if(autoCommand != null){
            autoCommand.schedule();
        }
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
        container.setAllBrake(true);
        container.init();

        if(autoCommand != null){
            autoCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void disabledInit() {
        container.disable();
    }
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void simulationInit() {}
    
    @Override
    public void simulationPeriodic() {
        container.simulationPeriodic();
        
        // calculate voltage sag due to current draw
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(container.getCurrentDraw()));

        SmartDashboard.putNumber("Sim/Total Current", container.getCurrentDraw());
        SmartDashboard.putNumber("Sim/Total Voltage", RobotController.getBatteryVoltage());
    }
}
