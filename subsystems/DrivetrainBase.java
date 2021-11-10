package org.frc5587.lib.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.frc5587.lib.pid.FPID;

public abstract class DrivetrainBase extends PIDSubsystem {
    // create leader and follower motors for the drivetrain
    protected WPI_TalonFX leftLeader; //, rightLeader, leftFollower, rightFollower;
    protected WPI_TalonFX rightLeader;
    protected WPI_TalonFX leftFollower;
    protected WPI_TalonFX rightFollower;

    // group the leader and follower motors so they can be controlled at the same time
    protected SpeedControllerGroup leftGroup; // = new SpeedControllerGroup(leftLeader, leftFollower);
    protected SpeedControllerGroup rightGroup; // = new SpeedControllerGroup(rightLeader, rightFollower);

    // make the speed controller groups into one drivetrain object
    protected DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

    protected AHRS ahrs;
    protected FPID turnFPID;
    protected double turnFPIDThrottle;
    protected boolean invertGyro;

    // placeholders for a SparkMax-based drivetrain
    /*
    protected CANSparkMax leftLeader = new CANSparkMax(0, MotorType.kBrushless);
    protected CANSparkMax rightLeader = new CANSparkMax(1, MotorType.kBrushless);
    protected CANSparkMax leftFollower = new CANSparkMax(2, MotorType.kBrushless);
    protected CANSparkMax rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    */

    public DrivetrainBase(WPI_TalonFX leftLeader, WPI_TalonFX rightLeader, WPI_TalonFX leftFollower, WPI_TalonFX rightFollower, FPID turnFPID, double turnFPIDThrottle, boolean invertGyro) {
        super(new PIDController(turnFPID.kP, turnFPID.kI, turnFPID.kD));
        
        this.leftLeader = leftLeader;
        this.rightLeader = rightLeader;
        this.leftFollower = leftFollower;
        this.rightFollower = rightFollower;
        this.turnFPID = turnFPID;
        this.turnFPIDThrottle = turnFPIDThrottle;
        this.invertGyro = invertGyro;
        leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
        rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);
        configureMotors();
    }
    
    public abstract void configureMotors();

    // drive with a given throttle and curve (arcade drive)
    public void arcadeDrive(double throttle, double curve) {
        differentialDrive.arcadeDrive(throttle, curve, false);
    }

    // drive with a given throttle for each side of the robot (tank drive)
    public void tankDrive(double leftThrottle, double rightThrottle) {
        differentialDrive.tankDrive(leftThrottle, rightThrottle, false);
    }

    // a tank drive that sets the voltages of the motors
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(-leftVolts);
        rightGroup.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    // a reversed version of tankdrivevolts
    public void tankDriveVoltsReverse(double leftVolts, double rightVolts) {
        this.tankDriveVolts(-leftVolts, -rightVolts);
    }

    public double getHeading() {
        return ahrs.getAngle() * (invertGyro ? -1 : 1);
        // return gyro.getAngle();
    }

    // the next two methods make the class PIDSubsystem compliant. 
    @Override
    protected void useOutput(double output, double setpoint) {
        arcadeDrive(turnFPIDThrottle, output + Math.copySign(turnFPID.kF, output));
    }

    @Override
    protected double getMeasurement() {
        return getHeading();
    }

    // stops all motors
    public void stop() {
        differentialDrive.stopMotor();
    }
}