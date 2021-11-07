package org.frc5587.lib.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class DrivetrainBase {
    // create leader and follower motors for the drivetrain
    private final WPI_TalonFX leftLeader = new WPI_TalonFX(0);
    private final WPI_TalonFX rightLeader = new WPI_TalonFX(1);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(2);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(3);

    // placeholders for a SparkMax-based drivetrain
    /*
    private final CANSparkMax leftLeader = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    */

    // group the leader and follower motors so they can be controlled at the same time
    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftLeader, leftFollower);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightLeader, rightFollower);

    // make the speed controller groups into one drivetrain object
    private final DifferentialDrive differentialDrive = new DifferentialDrive(rightGroup, leftGroup);

    // sets the motor modes and their speeds to 0
    public void startTalons() {
        leftLeader.set(ControlMode.PercentOutput, 0);
        rightLeader.set(ControlMode.PercentOutput, 0);
        leftFollower.set(ControlMode.Follower, 0);
        rightFollower.set(ControlMode.Follower, 1);
    }

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

    // stops all motors
    public void stop() {
        differentialDrive.stopMotor();
    }
}