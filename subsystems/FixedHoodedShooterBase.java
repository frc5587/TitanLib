package org.frc5587.lib.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.pid.JRAD;
import org.frc5587.lib.pid.UNP;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FixedHoodedShooterBase extends SubsystemBase {
    protected CANSparkMax leadMotor;
    protected CANSparkMax followerMotor;
    protected CANEncoder encoder;
    protected boolean oneOrTwoMotors; // false for one, true for two
    protected ShooterFeedbackController feedbackController;
    protected UNP unp;
    protected boolean setpointEnabled = false;
    protected double setpointVelocityRPS = 0;
    protected double targetHeightFromShooter = 0;

    /**
     * Creates with the assumption of two shooter motors
     * 
     * @param leadMotorID     CAN ID of leader motor
     * @param followerMotorID CAN ID of follower motor
     */
    public FixedHoodedShooterBase(int leadMotorID, int followerMotorID) {
        this(leadMotorID);

        oneOrTwoMotors = true;
        followerMotor = new CANSparkMax(followerMotorID, MotorType.kBrushless);
        followerMotor.follow(leadMotor);

        configureFollowerSpark();
    }

    /**
     * Creates with the assumption of one shooter motor
     * 
     * @param leadMotorID CAN ID of motor
     */
    public FixedHoodedShooterBase(int leadMotorID) {
        super();

        oneOrTwoMotors = false;
        leadMotor = new CANSparkMax(leadMotorID, MotorType.kBrushless);

        encoder = leadMotor.getEncoder();

        configureLeaderSpark();
    }

    /**
     * Sets JRAD constants, should be done in constructor
     */
    protected void setJRAD(JRAD jrad) {
        feedbackController = new ShooterFeedbackController(jrad, this::getVelocityRPS);
    }

    /**
     * Sets UNP regression constants, should be done in constructor
     */
    protected void setUNP(UNP unp) {
        this.unp = unp;
    }

    /**
     * If the JRAD control in enabled, it will update the motors based on the
     * setpoint
     */
    @Override
    public void periodic() {
        if (setpointEnabled) {
            setVoltage(feedbackController.setSetpointAndCalculate(setpointVelocityRPS));
        }

    }

    /**
     * Calculates shooter wheel velocity for a given distance using the UNP
     * regression constants
     * 
     * @param distance distance from target - METERS
     * @return velocity flywheel needs to spin up to - ROTATIONS PER SECOND
     */
    public double calculateVelocityRPSForDistance(double distance) {
        return (unp.U * distance) + (unp.P / (Math.pow(distance, 2) - unp.N));
    }

    /**
     * Calculated the optimal velocity using the UNP regression constants and then
     * sets the setpoint. It will only spin the shooter up if JRAD control has been
     * enabled.
     * 
     * @param distance distance from target
     */
    public void calculateAndSetVelocity(double distance) {
        setVelocityRPS(calculateVelocityRPSForDistance(distance));
    }

    /**
     * Sets the setpoint, this will spin the shooter up to the setpoint if JRAD
     * control has been enabled
     * 
     * Note: if tuned properly, the JRAD will intentionally overshoot the velocity
     * (by around 15%, but not necessarily), this is done preemptively so that when
     * the ball exits the shooter, it is spinning at the correct velocity
     * 
     * @param velocityRPS velocity to spin to - ROTATIONS PER SECOND
     */
    public void setVelocityRPS(double velocityRPS) {
        setpointVelocityRPS = velocityRPS;
    }

    /**
     * Sets the voltage. The negative is to correct the direction
     * 
     * @param voltage volts, basically caps at 12
     */
    public void setVoltage(double voltage) {
        leadMotor.setVoltage(voltage);
    }

    /**
     * Resets feedback controller and enables JRAD control
     */
    public void enableJRADControl() {
        feedbackController.reset();
        setpointEnabled = true;
    }

    /**
     * Disables JRAD control and stops motors
     */
    public void disableJRADControl() {
        setpointEnabled = false;
        setThrottle(0);
    }

    /**
     * Sets throttle of shooter.
     * 
     * @param throttle throttle value, within [-1, 1]
     */
    public void setThrottle(double throttle) {
        if (!setpointEnabled) {
            leadMotor.set(throttle);
        }
    }

    /**
     * @return whether shooter is at the setpoint
     */
    public boolean atSetpoint() {
        return feedbackController.atSetpoint();
    }

    /**
     * Gets the velocity of the first motor, but they should be spinning together,
     * so it represents both of their speeds
     * 
     * @return velocity - ROTATIONS PER SECOND
     */
    protected abstract double getVelocityRPS();

    protected abstract void configureLeaderSpark();

    protected abstract void configureFollowerSpark();
}
