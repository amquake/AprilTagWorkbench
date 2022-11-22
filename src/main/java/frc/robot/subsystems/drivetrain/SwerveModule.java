package frc.robot.subsystems.drivetrain;

import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

import java.util.Random;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.RobotConstants.*;
import frc.robot.subsystems.drivetrain.SwerveConstants.Module;

public class SwerveModule {

    // Module Constants
    private final Module moduleConstants;

    // Hardware
    private final PWMSparkMax driveMotor;
    private final Encoder driveEncoder;
    private final PWMSparkMax steerMotor;
    private final Encoder steerEncoder;

    // Control
    private SwerveModuleState lastDesiredState = new SwerveModuleState();
    private boolean openLoop = false;

    private PIDController drivePidController = new PIDController(kDriveKP, kDriveKI, kDriveKD);
    private ProfiledPIDController steerPidController = new ProfiledPIDController(
        kSteerKP, kSteerKI, kSteerKD,
        new Constraints(kSteerVelocity, kSteerAcceleration)
    );

    public SwerveModule(Module moduleConstants){
        this.moduleConstants = moduleConstants;

        driveMotor = new PWMSparkMax(moduleConstants.driveMotorID);
        driveMotor.setInverted(kInvertDrive);
        driveEncoder = new Encoder(moduleConstants.driveEncoderA, moduleConstants.driveEncoderB, kInvertDrive);
        driveEncoder.setDistancePerPulse(kDriveDistPerPulse);
        steerMotor = new PWMSparkMax(moduleConstants.steerMotorID);
        steerMotor.setInverted(kInvertSteer);
        steerEncoder = new Encoder(moduleConstants.steerEncoderA, moduleConstants.steerEncoderB, kInvertSteer);
        steerEncoder.setDistancePerPulse(kSteerRadPerPulse);

        // Simulation
        driveEncoderSim = new EncoderSim(driveEncoder);
        steerEncoderSim = new EncoderSim(steerEncoder);

        steerPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic(){
        // perform onboard PID to steer the module to the target angle
        TrapezoidProfile.State lastSteerSetpoint = steerPidController.getSetpoint();
        double steerPid = steerPidController.calculate(getAbsoluteHeading().getRadians(), lastDesiredState.angle.getRadians());
        double steerFF = kSteerFF.calculate(
            lastSteerSetpoint.velocity,
            steerPidController.getSetpoint().velocity,
            0.02
        );
        steerMotor.setVoltage(steerFF + steerPid);

        // perform onboard PID with inputted feedforward to drive the module to the target velocity
        double driveFF = kDriveFF.calculate(lastDesiredState.speedMetersPerSecond);
        double drivePid = 0;
        if(!openLoop){
            drivePid = drivePidController.calculate(driveEncoder.getRate(), lastDesiredState.speedMetersPerSecond);
        }
        driveMotor.setVoltage(driveFF + drivePid);
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop, boolean steerInPlace){
        Rotation2d currentRotation = getAbsoluteHeading();
        // avoid turning more than 90 degrees by inverting speed on large angle changes
        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);

        // if the module is not driving, maintain last angle setpoint
        if(!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01){
            desiredState.angle = lastDesiredState.angle;
        }

        this.lastDesiredState = desiredState;
    }

    public void setDriveBrake(boolean is){
    }
    public void setSteerBrake(boolean is){
    }

    /**
     * Module heading reported by steering cancoder
     */
    public Rotation2d getAbsoluteHeading(){
        return Rotation2d.fromRadians(steerEncoder.getDistance()).plus(new Rotation2d());
    }
    /**
     * @return State describing absolute module rotation and velocity in meters per second
     */
    public SwerveModuleState getAbsoluteState(){
        return new SwerveModuleState(
            driveEncoder.getRate(),
            getAbsoluteHeading()
        );
    }
    public SwerveModuleState getPerfAbsoluteState() {
        return new SwerveModuleState(
            perfDriveVelocity,
            getAbsoluteHeading()
        );
    }

    public void resetPosition(){
        driveEncoder.reset();
        perfDriveDistance = driveEncoder.getDistance();
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getDistance(),
            getAbsoluteHeading()
        );
    }
    public SwerveModulePosition getPerfPosition() {
        return new SwerveModulePosition(
            perfDriveDistance,
            getAbsoluteHeading()
        );
    }

    /**
     * @return Constants about this module, like motor IDs, cancoder angle offset, and translation from center
     */
    public Module getModuleConstants(){
        return moduleConstants;
    }

    public void log(){
        SwerveModuleState state = getAbsoluteState();
        int num = moduleConstants.moduleNum;
        SmartDashboard.putNumber("Module "+num+"/Steer Degrees", state.angle.getDegrees());
        SmartDashboard.putNumber("Module "+num+"/Steer Target Degrees", Math.toDegrees(steerPidController.getSetpoint().position));
        SmartDashboard.putNumber("Module "+num+"/Drive Velocity Feet", Units.metersToFeet(state.speedMetersPerSecond));
        SmartDashboard.putNumber("Module "+num+"/Drive Velocity Target Feet", Units.metersToFeet(lastDesiredState.speedMetersPerSecond));
        SmartDashboard.putNumber("Module "+num+"/Drive Current", getDriveCurrentDraw());
        SmartDashboard.putNumber("Module "+num+"/Steer Current", getSteerCurrentDraw());
    }


    // Simulation
    private final FlywheelSim driveWheelSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            kDriveFF.kv * kWheelCircumference / (2*Math.PI),
            kDriveFF.ka * kWheelCircumference / (2*Math.PI)
        ),
        DCMotor.getFalcon500(1),
        kDriveGearRatio
    );
    private final EncoderSim driveEncoderSim;
    private Random rand = new Random();
    private double perfDriveDistance = 0;
    private double perfDriveVelocity = 0;
    private final double kDriveVelocityNoiseRatio = 0.04; // scaled with velocity
    private final double kDriveAccelNoiseRatio = 0.06; // scaled with accel, added to velocity noise
    private final FlywheelSim steeringSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
        DCMotor.getFalcon500(1),
        kSteerGearRatio
    );
    private final EncoderSim steerEncoderSim;

    public void simulationPeriodic(){
        // apply our commanded voltage to our simulated physics mechanisms
        double driveVoltage = driveMotor.get() * RobotController.getBatteryVoltage();
        if(driveVoltage >= 0) driveVoltage = Math.max(0, driveVoltage-kSteerFF.ks);
        else driveVoltage = Math.min(0, driveVoltage+kSteerFF.ks);
        if(DriverStation.isDisabled()) driveVoltage = 0;
        driveWheelSim.setInputVoltage(driveVoltage);

        double steerVoltage = steerMotor.get() * RobotController.getBatteryVoltage();
        if(steerVoltage >= 0) steerVoltage = Math.max(0, steerVoltage-kSteerFF.ks);
        else steerVoltage = Math.min(0, steerVoltage+kSteerFF.ks);
        if(DriverStation.isDisabled()) steerVoltage = 0;
        steeringSim.setInputVoltage(steerVoltage);

        driveWheelSim.update(kDT);
        steeringSim.update(kDT);

        // update our simulated devices with our simulated physics results
        double perfDriveAccel = -perfDriveVelocity;
        perfDriveVelocity = driveWheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI) * kWheelCircumference;
        perfDriveAccel += perfDriveVelocity;
        perfDriveAccel /= kDT;
        double velocityNoise = Math.abs(perfDriveVelocity*kDriveVelocityNoiseRatio);
        velocityNoise += Math.abs(perfDriveAccel*kDriveAccelNoiseRatio);
        driveEncoderSim.setRate(rand.nextGaussian(perfDriveVelocity, velocityNoise));
        perfDriveDistance = perfDriveDistance + perfDriveVelocity * kDT;
        driveEncoderSim.setDistance(driveEncoderSim.getDistance() + driveEncoderSim.getRate() * kDT);

        steerEncoderSim.setRate(steeringSim.getAngularVelocityRadPerSec());
        steerEncoderSim.setDistance(steerEncoderSim.getDistance() + steerEncoderSim.getRate() * kDT);
    }

    public double getDriveCurrentDraw(){
        return driveWheelSim.getCurrentDrawAmps()/10;
    }
    public double getSteerCurrentDraw(){
        return steeringSim.getCurrentDrawAmps()/10;
    }
}