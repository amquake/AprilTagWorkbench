package frc.robot.subsystems.drivetrain;

import static frc.robot.auto.AutoConstants.*;

import java.util.Random;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotConstants.*;
import frc.robot.auto.AutoConstants;
import frc.robot.util.LogUtil;

public class SwerveDrive extends SubsystemBase {

    // construct our modules in order with their specific constants
    private final SwerveModule[] swerveMods = {
        new SwerveModule(SwerveConstants.Module.FL),
        new SwerveModule(SwerveConstants.Module.FR),
        new SwerveModule(SwerveConstants.Module.BL),
        new SwerveModule(SwerveConstants.Module.BR)
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        swerveMods[0].getModuleConstants().centerOffset,
        swerveMods[1].getModuleConstants().centerOffset,
        swerveMods[2].getModuleConstants().centerOffset,
        swerveMods[3].getModuleConstants().centerOffset
    );

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveOdometry perfOdometry;
    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
    private boolean isFieldRelative = true;

    // path controller and its dimension-specific controllers
    // i.e 1 meter error in the x direction = kP meters per second x velocity added
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    // our auto rotation targets are profiled to obey velocity and acceleration constraints
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kDThetaController,
        AutoConstants.kThetaControllerConstraints
    );
    // our auto controller which follows trajectories and adjusts target chassis speeds to reach a desired pose
    private final HolonomicDriveController pathController = new HolonomicDriveController(xController, yController, thetaController);

    private Trajectory logTrajectory;
    
    public SwerveDrive() {
                
        zeroGyro();
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(kThetaPositionTolerance, kThetaVelocityTolerance);
        pathController.setEnabled(true); // disable for feedforward-only auto

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );
        perfOdometry = new SwerveDriveOdometry(kinematics, getPerfGyroYaw(), getPerfModulePositions());

        gyroSim = new ADXRS450_GyroSim(gyro);
    }

    @Override
    public void periodic() {
        for(SwerveModule module : swerveMods) {
            module.periodic();
        }

        // display our robot (and individual modules) pose on the field
        poseEstimator.update(getGyroYaw(), getModulePositions());
        perfOdometry.update(getPerfGyroYaw(), getPerfModulePositions());
    }

    /**
     * Basic teleop drive control; ChassisSpeeds values representing vx, vy, and omega
     * are converted to individual module states for the robot to follow
     * @param vxMeters x velocity (forward)
     * @param vyMeters y velocity (strafe)
     * @param omegaRadians angular velocity (rotation CCW+)
     * @param openLoop If swerve modules should not use velocity PID
     */
    public void drive(double vxMeters, double vyMeters, double omegaRadians, boolean openLoop){
        double vx = vxMeters;
        double vy = vyMeters;
        double omega = omegaRadians;
        ChassisSpeeds targetChassisSpeeds;
        if(isFieldRelative){
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getHeading());
        }
        else{
            targetChassisSpeeds = new ChassisSpeeds(vx,vy,omega);
        }
        setChassisSpeeds(targetChassisSpeeds, openLoop, false);
    }
    /**
     * Drive control using angle position (theta) instead of velocity (omega).
     * The {@link #thetaController theta PID controller} calculates an angular velocity in order
     * to reach the target angle, making this method similar to autonomous path following without
     * x/y position controllers. This method assumes field-oriented control and is not affected
     * by the value of {@link #isFieldRelative}.
     * @param vxMeters x velocity (forward)
     * @param vyMeters y velocity (strafe)
     * @param targetRotation target angular position
     * @param openLoop If swerve modules should not use velocity PID
     * @return If the drivetrain rotation is within tolerance of the target rotation
     */
    public boolean drive(double vxMeters, double vyMeters, Rotation2d targetRotation, boolean openLoop){
        // rotation speed
        double rotationRadians = getPose().getRotation().getRadians();
        double pidOutput = thetaController.calculate(rotationRadians, targetRotation.getRadians());

        // + translation speed
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMeters,
            vyMeters,
            pidOutput,
            getHeading()
        );

        setChassisSpeeds(targetChassisSpeeds, openLoop, false);
        return thetaController.atGoal();
    }
    /**
     * Drive control intended for path following utilizing the {@link #pathController path controller}.
     * This method always uses closed-loop control on the modules.
     * @param targetState Trajectory state containing target translation and velocities
     * @param targetRotation Target rotation independent of trajectory motion
     */
    public void drive(Trajectory.State targetState, Rotation2d targetRotation){
        // determine ChassisSpeeds from path state and positional feedback control from HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
            getPose(),
            targetState,
            targetRotation
        );
        // command robot to reach the target ChassisSpeeds
        setChassisSpeeds(targetChassisSpeeds, false, false);
    }

    /**
     * Command the swerve modules to the desired states.
     * Velocites above maximum speed will be downscaled (preserving ratios between modules)
     * @param steerInPlace If modules should steer to target angle when target velocity is 0
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxLinearSpeed);
        for(int i=0;i<4;i++){
            swerveMods[i].setDesiredState(desiredStates[i], openLoop, steerInPlace);
        }
    }
    /**
     * Uses kinematics to convert ChassisSpeeds to module states.
     * @see {@link #setModuleStates(SwerveModuleState[], boolean)}
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace){
        setModuleStates(kinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
        this.targetChassisSpeeds = targetChassisSpeeds;
    }
    public void stop(){
        drive(0, 0, 0, true);
    }
    public CommandBase stopC() {
        return new InstantCommand(this::stop, this);
    }

    /**
     * Changes whether drive methods use field or robot-oriented control.
     */
    public void setIsFieldRelative(boolean is) {isFieldRelative = is;}

    public void setBrakeOn(boolean is){
        for(SwerveModule module : swerveMods) {
            module.setDriveBrake(is);
            module.setSteerBrake(is);
        }
    }

    public void zeroGyro(){
        gyro.reset();
        perfGyroAngle = new Rotation2d();
    }
    public void addVisionMeasurement(Pose2d measurement, double latencySeconds){
        poseEstimator.addVisionMeasurement(
            measurement,
            Timer.getFPGATimestamp() - latencySeconds
        );
    }
    public void addVisionMeasurement(Pose2d measurement, double latencySeconds, Matrix<N3, N1> stdDevs){
        poseEstimator.addVisionMeasurement(
            measurement,
            Timer.getFPGATimestamp() - latencySeconds,
            stdDevs
        );
    }
    public void resetOdometry(Pose2d pose){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        perfOdometry.resetPosition(getPerfGyroYaw(), getPerfModulePositions(), pose);
    }
    public void resetNoisyOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    public void resetPathController(){
        xController.reset();
        yController.reset();
        thetaController.reset(getHeading().getRadians(), getChassisSpeeds().omegaRadiansPerSecond);
    }

    public boolean getIsFieldRelative() {return isFieldRelative;}

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    public Pose2d getPose(double secondsAgo) {
        return poseEstimator.getEstimatedPosition(secondsAgo);
    }
    public Pose2d getPerfPose() {
        return perfOdometry.getPoseMeters();
    }
    /**
     * Swerve drive rotation on the field reported by odometry.
     */
    public Rotation2d getHeading(){
        return getPose().getRotation();
    }
    /**
     * Raw gyro yaw (this may not match the field heading!).
     */
    public Rotation2d getGyroYaw(){
        return gyro.getRotation2d();
    }
    public Rotation2d getPerfGyroYaw() {
        return perfGyroAngle;
    }

    public double getLinearVelocity(){
        return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
    }
    public double getMaxLinearVelocityMeters(){
        return SwerveConstants.kMaxLinearSpeed;
    }
    public double getMaxAngularVelocityRadians(){
        return SwerveConstants.kMaxAngularSpeed;
    }
    
    /**
     * @return An ordered array filled with module states (rotation, velocity)
     */
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            swerveMods[0].getAbsoluteState(),
            swerveMods[1].getAbsoluteState(),
            swerveMods[2].getAbsoluteState(),
            swerveMods[3].getAbsoluteState()
        };
    }
    public SwerveModuleState[] getPerfModuleStates(){
        return new SwerveModuleState[]{
            swerveMods[0].getPerfAbsoluteState(),
            swerveMods[1].getPerfAbsoluteState(),
            swerveMods[2].getPerfAbsoluteState(),
            swerveMods[3].getPerfAbsoluteState()
        };
    }
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            swerveMods[0].getPosition(),
            swerveMods[1].getPosition(),
            swerveMods[2].getPosition(),
            swerveMods[3].getPosition()
        };
    }
    public SwerveModulePosition[] getPerfModulePositions(){
        return new SwerveModulePosition[]{
            swerveMods[0].getPerfPosition(),
            swerveMods[1].getPerfPosition(),
            swerveMods[2].getPerfPosition(),
            swerveMods[3].getPerfPosition()
        };
    }
    /**
     * @return An ordered array filled with the module field poses 
     */
    public Pose2d[] getModulePoses(){
        Pose2d[] modulePoses = new Pose2d[4];
        for(int i=0;i<4;i++){
            SwerveModule module = swerveMods[i];
            modulePoses[i] = getPose().transformBy(
                new Transform2d(module.getModuleConstants().centerOffset, module.getAbsoluteHeading())
            );
        }
        return modulePoses;
    }
    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }
    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }
    public ChassisSpeeds getPerfChassisSpeeds() {
        return kinematics.toChassisSpeeds(getPerfModuleStates());
    }

    public HolonomicDriveController getPathController(){
        return pathController;
    }

    public void log(){
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Drive/Heading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive/X", pose.getX());
        SmartDashboard.putNumber("Drive/Y", pose.getY());
        SmartDashboard.putNumberArray("Drive/PoseArray", LogUtil.toPoseArray(pose));
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        SmartDashboard.putNumber("Drive/Target Heading", Math.toDegrees(thetaController.getSetpoint().position));
        SmartDashboard.putNumber("Drive/VX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/VY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Omega Degrees", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Drive/Target VX", targetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target VY", targetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
        
        for(SwerveModule module : swerveMods) {
            module.log();
        }
    }
    public void logTrajectory(Trajectory trajectory) {logTrajectory = trajectory;}
    public Trajectory getLogTrajectory() {return logTrajectory;}



    //----- Simulation
    private final ADXRS450_GyroSim gyroSim; // simulate gyro
    private static final Random rand = new Random();
    private static final double kGyroNoiseStdDevRadians = (1.0 / 2000.0) * (kDT / 0.02);
    private Rotation2d perfGyroAngle = new Rotation2d();
    private Rotation2d perfGyroRate = new Rotation2d();

    @Override
    public void simulationPeriodic(){
        for(SwerveModule module : swerveMods) {
            module.simulationPeriodic();
        }

        double perfChassisOmega = getPerfChassisSpeeds().omegaRadiansPerSecond;
        perfGyroRate = new Rotation2d(perfChassisOmega*kDT);
        gyroSim.setRate(perfGyroRate.unaryMinus().getDegrees());
        gyroSim.setAngle(gyro.getAngle() + perfGyroRate.unaryMinus().getDegrees());
        gyroSim.setAngle(rand.nextGaussian(gyro.getAngle(), Math.toDegrees(kGyroNoiseStdDevRadians)));
        
        perfGyroAngle = perfGyroAngle.plus(perfGyroRate);
    }

    public double getCurrentDraw(){
        double sum = 0;
        for(SwerveModule module : swerveMods) sum += module.getDriveCurrentDraw() + module.getSteerCurrentDraw();
        return sum;
    }
}
