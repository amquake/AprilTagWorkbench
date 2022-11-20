/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Custom {@link XboxController} wrapper to add convenience features for
 * driving.
 */
public class OCXboxController extends XboxController {

    // Pre-made buttons to reduce verbosity
    public final JoystickButton aButton;
    public final JoystickButton bButton;
    public final JoystickButton xButton;
    public final JoystickButton yButton;
    public final JoystickButton leftBumper;
    public final JoystickButton rightBumper;
    public final JoystickButton backButton;
    public final JoystickButton startButton;
    public final JoystickButton leftStick;
    public final JoystickButton rightStick;
    public final edu.wpi.first.wpilibj2.command.button.Button leftTriggerButton; // curse disambiguation
    public final edu.wpi.first.wpilibj2.command.button.Button rightTriggerButton;
    public final POVButton povUpButton;
    public final POVButton povRightButton;
    public final POVButton povDownButton;
    public final POVButton povLeftButton;

    private static final double kDeadband = 0.12;

    public static final double kSpeedDefault = .55; //Traingins peed - 0.3 OG speed - 0.55
    public static final double kSpeedFast = 0.65;
    public static final double kSpeedMax = 0.8;
    
    private double drivespeed = kSpeedDefault;
    private static final double kTurnDrivespeed = 0.5; //Og speed 0.6 

    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(1.0 / 0.6); // 1 / x seconds to full 0.5 
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(1.0 / 0.6); // 0.5
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(1 / 0.33); //0.33

    /**
     * Constructs XboxController on DS joystick port.
     */
    public OCXboxController(int port) {
        super(port);

        aButton = new JoystickButton(this, XboxController.Button.kA.value);
        bButton = new JoystickButton(this, XboxController.Button.kB.value);
        xButton = new JoystickButton(this, XboxController.Button.kX.value);
        yButton = new JoystickButton(this, XboxController.Button.kY.value);
        leftBumper = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(this, XboxController.Button.kRightBumper.value);
        backButton = new JoystickButton(this, XboxController.Button.kBack.value);
        startButton = new JoystickButton(this, XboxController.Button.kStart.value);
        leftStick = new JoystickButton(this, XboxController.Button.kLeftStick.value);
        rightStick = new JoystickButton(this, XboxController.Button.kRightStick.value);
        leftTriggerButton = new edu.wpi.first.wpilibj2.command.button.Button(() -> getLeftTriggerAxis() > 0.15);
        rightTriggerButton = new edu.wpi.first.wpilibj2.command.button.Button(() -> getRightTriggerAxis() > 0.15);
        povUpButton = new POVButton(this, 0);
        povRightButton = new POVButton(this, 90);
        povDownButton = new POVButton(this, 180);
        povLeftButton = new POVButton(this, 270);
    }

    /**
     * Deadbands a value, re-scales it, and applies a power.
     * 
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value, double exp) {
        value = MathUtil.applyDeadband(value, kDeadband);
        return Math.signum(value) * Math.pow(Math.abs(value), exp);
    }

    public void setDriveSpeed(double drivespeed) {
        this.drivespeed = drivespeed;
    }

    public double getDriveSpeed() {
        return drivespeed;
    }

    @Override
    public double getLeftY() {
        return getLeftY(1);
    }
    public double getLeftY(double exponent) {
        return -scaledPowerDeadband(super.getLeftY(), exponent);
    }
    @Override
    public double getLeftX() {
        return getLeftX(1);
    }
    public double getLeftX(double exponent) {
        return -scaledPowerDeadband(super.getLeftX(), exponent);
    }
    @Override
    public double getRightY() {
        return getRightY(1);
    }
    public double getRightY(double exponent) {
        return -scaledPowerDeadband(super.getRightY(), exponent);
    }
    @Override
    public double getRightX() {
        return getRightX(1);
    }
    public double getRightX(double exponent) {
        return -scaledPowerDeadband(super.getRightX(), exponent);
    }

    /**
     * Applies deadband math and rate limiting to left Y to give 'forward' percentage.
     * Affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getForward() {
        return forwardLimiter.calculate(getLeftY(2) * drivespeed);
    }

    /**
     * Applies deadband math and rate limiting to left X to give 'strafe' percentage.
     * Affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getStrafe() {
        return strafeLimiter.calculate(getLeftX(2) * drivespeed);
    }

    /**
     * Applies deadband math and rate limiting to right X to give 'turn' percentage.
     * Not affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getTurn() {
        return turnLimiter.calculate(getRightX(2) * kTurnDrivespeed);
    }

    /**
     * Reset the slew-rate limiters on the joysticks
     */
    public void resetLimiters() {
        forwardLimiter.reset(0);
        strafeLimiter.reset(0);
        turnLimiter.reset(0);
    }

    public void rumble(double value){
        setRumble(RumbleType.kRightRumble, value);
        setRumble(RumbleType.kLeftRumble, value);
    }
    public void rumble(boolean left, double value){
        RumbleType side = left ? RumbleType.kLeftRumble : RumbleType.kRightRumble;
        setRumble(side, value);
    }
}
