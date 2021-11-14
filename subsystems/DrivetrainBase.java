package org.frc5587.lib.subsystems;

import org.frc5587.lib.advanced.LimitedPoseMap;
import org.frc5587.lib.pid.FPID;
import java.util.Hashtable;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public abstract class DrivetrainBase extends PIDSubsystem {
    // create leader and follower motors for the drivetrain, as well as encoders that will attach to the leaders.
    protected WPI_TalonFX leftLeader, rightLeader, leftFollower, rightFollower;
    protected Encoder leftEncoder, rightEncoder;

    // group the leader and follower motors so they can be controlled at the same time
    protected SpeedControllerGroup leftGroup, rightGroup;

    // make the speed controller groups into one drivetrain object
    protected DifferentialDrive differentialDrive;

    // create a hashtable to look up constants
    protected Hashtable<String, Object> constants;

    // create variables needed for odometry.
    protected AHRS ahrs;
    protected FPID turnFPID;
    protected double turnFPIDThrottle;
    protected boolean invertGyro;
    protected int invertLeft;
    protected int invertRight;
    protected DifferentialDriveOdometry odometry;
    private final LimitedPoseMap poseHistory;
    private double lastAngleSetpoint = Double.NaN;
    private final PIDController turnController;

    // set all of the variables from the subclass to this abstract class
    public DrivetrainBase(Hashtable<String, Object> constants, WPI_TalonFX...motors) {
        super(new PIDController((double) ((FPID) constants.get("TURN_FPID")).kP, (double) ((FPID) constants.get("TURN_FPID")).kI, (double) ((FPID) constants.get("TURN_FPID")).kD));        
        // disable PID control on start
        this.disable();
        this.constants = constants;
        // check how many motors were given
        int numMotors = motors.length;
        int i = 0;
        // make arrays to put each type of motor in
        WPI_TalonFX[] leaders = new WPI_TalonFX[2];
        WPI_TalonFX[] leftFollowers = new WPI_TalonFX[(motors.length/2)-1];
        WPI_TalonFX[] rightFollowers = new WPI_TalonFX[(motors.length/2)-1];
        for(WPI_TalonFX motor : motors) {
            // if the motor isn't the first motor in either half of the motors array, it's a follower.
            // if the motor is in the first half of the motors array, it's on the left side.
            if(!(i+1 == 1 || i+1 == numMotors/2) && ((i+1) < (numMotors/2))) {
                leftFollowers[i] = motor;
            }

            // if the motor is in the second half of the motors array, it goes on the right.
            else if(!(i+1 == 1 || i+1 == numMotors/2) && !((i+1) < (numMotors/2))) {
                rightFollowers[i] = motor;
            }

            // if the motor is the first motor in either half of the motors array, it's a leader.
            else {
                leaders[i] = motor;
            }
        }
        // the first leader will be the left one
        this.leftLeader = leaders[0];
        // the second leader will be the right one
        this.rightLeader = leaders[1];
        // set all variables declared at the top to those given in the constructor (mostly constants)
        var currentAngle = Rotation2d.fromDegrees(getHeading360());
        this.odometry = new DifferentialDriveOdometry(currentAngle);
        this.poseHistory =  new LimitedPoseMap((int) constants.get("HISTORY_LIMIT"));
        this.turnFPID = (FPID) constants.get("TURN_FPID");
        this.turnFPIDThrottle = (double) constants.get("TURN_PID_FORWARD_THROTTLE");
        this.invertGyro = (boolean) constants.get("INVERT_GYRO_DIRECTION");
        this.invertLeft = (boolean) constants.get("LEFT_SIDE_INVERTED") ? 1 : -1;
        this.invertRight = (boolean) constants.get("RIGHT_SIDE_INVERTED") ? 1 : -1;
        turnController = getController();
        turnController.enableContinuousInput(-180, 180);
        turnController.setIntegratorRange(-1, 1);
        turnController.setTolerance((double) constants.get("TURN_PID_TOLERANCE_DEG"));
        // make speedcontroller groups with the leader and follower motors we got earlier
        // if there are no followers, do not include the followers in the speedcontrollergroups.
        if(leftFollowers.length != 0) {
            leftGroup = new SpeedControllerGroup(leftLeader, leftFollowers);
        }
        else {
            leftGroup = new SpeedControllerGroup(leftLeader);
        }

        if(rightFollowers.length != 0) {
            rightGroup = new SpeedControllerGroup(rightLeader, rightFollowers);
        }
        else {
            rightGroup = new SpeedControllerGroup(rightLeader);
        }

        differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
        configureMotors();
    }
    
    // create a required method for subclasses
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
        leftGroup.setVoltage(invertLeft * leftVolts);
        rightGroup.setVoltage(invertRight * rightVolts);
        differentialDrive.feed();
    }

    // a reversed version of tankdrivevolts
    public void tankDriveVoltsReverse(double leftVolts, double rightVolts) {
        this.tankDriveVolts(-leftVolts, -rightVolts);
    }

    // NOTE: many of the following methods are self-explanatory. Only the neccesary documentation will be made.
    public double getLeftPositionRotation() {
        return leftLeader.getSelectedSensorPosition();
    }

    public double getRightPositionRotation() {
        return rightLeader.getSelectedSensorPosition();
    }

    private double rotationsToMeters(double rotations) {
        // number of rotations * circumference of wheel
        return rotations * (double) constants.get("WHEEL_DIAMETER_METERS") * Math.PI;
    }

    public double getLeftPositionMeters() {
        return rotationsToMeters(getLeftPositionRotation());
    }

    public double getRightPositionMeters() {
        return rotationsToMeters(getRightPositionRotation());
    }

    public double getLeftVelocityRPM() {
        return leftLeader.getSelectedSensorVelocity() / 2;
    }

    public double getRightVelocityRPM() {
        return rightLeader.getSelectedSensorVelocity() / 2;
    }

    // converts rotations per minute (RPM) to meters per second (MPS)
    private double rpmToMPS(double rotationsPerMinute) {
        var radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rotationsPerMinute);
        var linearMetersPerSecond = (double) radiansPerSecond * (double) constants.get("WHEEL_RADIUS_METERS");
        return linearMetersPerSecond;
    }

    public double getLeftVelocityMPS() {
        return rpmToMPS(getLeftVelocityRPM());
    }

    public double getRightVelocityMPS() {
        return rpmToMPS(getRightVelocityRPM());
    }

    // gets the average velocity of the robot by taking the mean of both sides
    public double getAbsoluteAverageVelocityMPS() {
        return (Math.abs(getRightVelocityMPS()) + Math.abs(getLeftVelocityMPS())) / 2;
    }

    // gets the direction of the robot
    public double getHeading() {
        return ahrs.getAngle() * (invertGyro ? -1 : 1);
        // return gyro.getAngle();
    }

    public double getHeading360() {
        return Math.IEEEremainder(getHeading(), 360.0/* d */);
    }

    public double getHeading180() {
        var heading = getHeading() % 360;
        if (heading > 180) {
            return heading - 360;
        } else if (heading < -180) {
            return heading + 360;
        } else {
            return heading;
        }
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMPS(), getRightVelocityMPS());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeedsReverse() {
        return new DifferentialDriveWheelSpeeds(-getLeftVelocityMPS(), -getRightVelocityMPS());
    }

    public Pose2d getClosestPoseAtTime(double time) {
        return poseHistory.getClosest(time);
    }

    // resets the positions of the encoders to 0
    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    // resets the heading of the robot
    public void resetHeading() {
        ahrs.reset();
    }

    // resets both the heading and the encoders, as well as the odometry of the robot
    public void resetOdometry() {
        resetHeading();
        resetEncoders();
        odometry.resetPosition(new Pose2d(), new Rotation2d());
    }

    // resets the odometry to a given position param
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, ahrs.getRotation2d());
    }

    public double getAverageEncoderDistance() {
        return (leftLeader.getSelectedSensorPosition() + rightLeader.getSelectedSensorPosition()) / 2;
    }

    public double getTurnRate() {
        return -ahrs.getRate();
    }

    // sets the idle modes of all motors
    public void setNeutralModes(NeutralMode idleMode) {
        leftLeader.setNeutralMode(idleMode);
        leftFollower.setNeutralMode(idleMode);
        rightLeader.setNeutralMode(idleMode);
        rightFollower.setNeutralMode(idleMode);
    }

    // checks if the pidcontroller is at the last setpoint
    public boolean atSetpoint() {
        return turnController.atSetpoint();
    }

    // gets the last setpoint of the pidcontroller
    public double getLastAngleSetpoint() {
        return lastAngleSetpoint;
    }
    
    // the next few methods override PIDSubsystem. 
    @Override
    public void periodic() {
        // Call periodic of PIDSubsystem to ensure PID controller runs
        super.periodic();

        // Update the pose
        var gyroAngle = Rotation2d.fromDegrees(getHeading360());
        odometry.update(gyroAngle, getLeftPositionMeters(), getRightPositionMeters());
        var translation = odometry.getPoseMeters().getTranslation();

        // Log the pose
        poseHistory.put(Timer.getFPGATimestamp(), getPose());
    }

    // sets the setpoint of the pidcontroller
    @Override
    public void setSetpoint(double setpoint) {
        lastAngleSetpoint = setpoint;
        super.setSetpoint(setpoint);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        arcadeDrive(turnFPIDThrottle, output + Math.copySign(turnFPID.kF, output));
    }

    @Override
    protected double getMeasurement() {
        return getHeading();
    }

    // stops the drivetrain's speedcontrollergroups rather than the differentialdrive
    public void stopDrivetrain() {
        leftGroup.set(0);
        rightGroup.set(0);
    }

    // sets the speeds of the speedcontrollergroups rather than the differentrialdrive
    public void setDrive(double speed) {
        leftGroup.set(invertLeft * speed);
        rightGroup.set(invertRight * speed);
    }

    // stops all motors
    public void stop() {
        differentialDrive.stopMotor();
    }
}
