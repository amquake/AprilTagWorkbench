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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom {@link XboxController} wrapper to add convenience features for
 * driving.
 */
public class OCXboxController extends CommandXboxController {

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

    /** Rumble both controller sides */
    public void rumble(double value){
        getHID().setRumble(RumbleType.kRightRumble, value);
        getHID().setRumble(RumbleType.kLeftRumble, value);
    }
    /** Rumble one controller side */
    public void rumble(boolean left, double value){
        RumbleType side = left ? RumbleType.kLeftRumble : RumbleType.kRightRumble;
        getHID().setRumble(side, value);
    }

    // ----- Trigger factories

    /**
     * Constructs an event instance around the left trigger passing a threshold.
     *
     * @param threshold Trigger axis values greater than this will trigger the event
     * @return an event instance representing the left trigger exceeding the threshold's digital attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftTrigger(EventLoop)
     */
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
    }
    /**
     * Constructs an event instance around the left trigger passing a threshold.
     *
     * @param loop the event loop instance to attach the event to.
     * @param threshold Trigger axis values greater than this will trigger the event
     * @return an event instance representing the left trigger exceeding the threshold's digital signal
     *     attached to the given loop.
     */
    public Trigger leftTrigger(EventLoop loop, double threshold) {
        return new BooleanEvent(loop, () -> getLeftTriggerAxis() > threshold).castTo(Trigger::new);
    }
    /**
     * Constructs an event instance around the right trigger passing a threshold.
     *
     * @param threshold Trigger axis values greater than this will trigger the event
     * @return an event instance representing the right trigger exceeding the threshold's digital attached
     *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightTrigger(EventLoop)
     */
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
    }
    /**
     * Constructs an event instance around the right trigger passing a threshold.
     *
     * @param loop the event loop instance to attach the event to.
     * @param threshold Trigger axis values greater than this will trigger the event
     * @return an event instance representing the right trigger exceeding the threshold's digital signal
     *     attached to the given loop.
     */
    public Trigger rightTrigger(EventLoop loop, double threshold) {
        return new BooleanEvent(loop, () -> getRightTriggerAxis() > threshold).castTo(Trigger::new);
    }
}
