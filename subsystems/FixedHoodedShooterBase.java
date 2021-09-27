package org.frc5587.lib.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.controllers.UnifiedShooterController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FixedHoodedShooterBase extends SubsystemBase {
    protected CANSparkMax leadMotor;
    protected CANSparkMax followerMotor;
    protected CANEncoder encoder;
    protected boolean hasTwoMotors; // false for one, true for two motors
    protected UnifiedShooterController shooterController;
    protected boolean setpointEnabled = false;
    protected double distanceFromTarget = 5;
    protected double targetHeightFromShooter = 0;

    /**
     * Creates with the assumption of two shooter motors
     * 
     * @param leadMotorID     CAN ID of leader motor
     * @param followerMotorID CAN ID of follower motor
     */
    public FixedHoodedShooterBase(int leadMotorID, int followerMotorID) {
        this(leadMotorID);

        hasTwoMotors = true;
        followerMotor = new CANSparkMax(followerMotorID, MotorType.kBrushless);
        // followerMotor.follow(leadMotor);

        
        encoder = followerMotor.getEncoder();
        encoder.setPosition(0);

        configureFollowerSpark();
    }

    /**
     * Creates with the assumption of one shooter motor
     * 
     * @param leadMotorID CAN ID of motor
     */
    public FixedHoodedShooterBase(int leadMotorID) {
        super();

        hasTwoMotors = false;
        leadMotor = new CANSparkMax(leadMotorID, MotorType.kBrushless);


        configureLeaderSpark();
    }

    /**
     * Sets the shooter controller, should be done in constructor of implemented class
     */
    protected void setShooterController(UnifiedShooterController shooterController) {
        this.shooterController = shooterController; 
    }

    /**
     * If distance control is enabled, it will update the motors based on the distance from the target
     */
    @Override
    public void periodic() {
        if (setpointEnabled) {
            setVoltage(shooterController.setDistanceAndCalculate(distanceFromTarget));
        }

    }

    /**
     * Sets the distance the shooter is from the target.
     * This will spin the shooter up if control is enabled
     * 
     * @param distanceFromTarget distance from the target - METERS
     */
    public void setDistanceFromTarget(double distanceFromTarget) {
        this.distanceFromTarget = distanceFromTarget;
    }

    /**
     * Sets the voltage. The negative is to correct the direction
     * 
     * @param voltage volts, basically caps at 12
     */
    public void setVoltage(double voltage) {
        leadMotor.setVoltage(voltage);
        followerMotor.setVoltage(voltage);
    }

    /**
     * Resets shooter controller and enables distance control
     */
    public void enableDistanceControl() {
        shooterController.reset();
        setpointEnabled = true;
    }

    /**
     * Disables distance control and stops motors
     */
    public void disableDistanceControl() {
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
            followerMotor.set(throttle);
        }
    }

    /**
     * @return whether shooter is at the setpoint
     */
    public boolean atSetpoint() {
        return shooterController.atSetpoint();
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
