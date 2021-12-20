package org.frc5587.lib.subsystems;

import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.advanced.LimitedPoseMap;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public abstract class DrivetrainBase extends PIDSubsystem {
    // create leader and follower motors for the drivetrain
    protected WPI_TalonFX leftLeader, rightLeader;
    protected WPI_TalonFX[] leftFollowers;
    protected WPI_TalonFX[] rightFollowers;

    // group the leader and follower motors so they can be controlled at the same time
    protected SpeedControllerGroup leftGroup, rightGroup;

    // make the speed controller groups into one drivetrain object
    protected DifferentialDrive differentialDrive;

    // create a hashtable to look up constants
    protected DriveConstants constants;

    // create variables needed for odometry.
    protected AHRS ahrs = new AHRS();
    protected FPID turnFPID;
    protected double turnFPIDThrottle;
    protected boolean invertGyro;
    protected DifferentialDriveOdometry odometry;
    private final LimitedPoseMap poseHistory;
    private double lastAngleSetpoint = Double.NaN;
    private final PIDController turnController;

    public static class DriveConstants {
        public final FPID turnFPID;
        public final double turnPIDThrottle, turnPIDToleranceDeg, wheelDiameterMeters;
        public final int historyLimit;
        public final boolean invertGyro;
        public final int epr;
        public final double gearing;

        public DriveConstants(FPID turnFPID, double turnPIDThrottle, double turnPIDToleranceDeg, double wheelDiameterMeters, int historyLimit, boolean invertGyro, int epr, double gearing) {
            this.turnFPID = turnFPID;
            this.turnPIDThrottle = turnPIDThrottle;
            this.turnPIDToleranceDeg = turnPIDToleranceDeg;
            this.wheelDiameterMeters = wheelDiameterMeters;
            this.historyLimit = historyLimit;
            this.invertGyro = invertGyro;
            this.epr = epr;
            this.gearing = gearing;
        }
    }

    // set all of the variables from the subclass to this abstract class
    // MotorIDs: a arrays of integer CAN IDs used to make motors. index 0 should ALWAYS be the leader motor, and anything else is a follower.
    public DrivetrainBase(DriveConstants constants, int[] leftMotorIDs, int[] rightMotorIDs) {
        /* PID SETUP */
        super(new PIDController(constants.turnFPID.kP, constants.turnFPID.kI, constants.turnFPID.kD));        
        // disable PID control on start
        this.disable();

        /* VARIABLE DECLARATION */
        // set all variables declared at the top to those given in the constructor (mostly constants)
        this.constants = constants;
        Rotation2d currentAngle = Rotation2d.fromDegrees(getHeading360());
        this.odometry = new DifferentialDriveOdometry(currentAngle);
        this.poseHistory =  new LimitedPoseMap(constants.historyLimit);
        this.turnFPID = constants.turnFPID;
        this.turnFPIDThrottle = constants.turnPIDThrottle;
        this.invertGyro = constants.invertGyro;
        turnController = getController();
        turnController.enableContinuousInput(-180, 180);
        turnController.setIntegratorRange(-1, 1);
        turnController.setTolerance(constants.turnPIDToleranceDeg);

        /* MOTOR SETUP */
        // use arrays to contain each follower motor
        leftFollowers = new WPI_TalonFX[leftMotorIDs.length-1];
        rightFollowers = new WPI_TalonFX[rightMotorIDs.length-1];
        // put each motor in an array corresponding to its type (leader or follower)
        for(int i = 0; i < leftMotorIDs.length; i++) {
            // if it's the first one in the array, it's a leader
            if(i == 0) {
                // make a new motor with the given ID and put it in the array
                this.leftLeader = new WPI_TalonFX(leftMotorIDs[i]);
            }
            // otherwise it's a follower
            else {
                leftFollowers[i-1] = new WPI_TalonFX(leftMotorIDs[i]);
            }
        }
        // do the same for the right motors
        for(int i = 0; i < rightMotorIDs.length; i++) {
            if(i == 0) {
                this.rightLeader = new WPI_TalonFX(rightMotorIDs[i]);
            }
            else {
                rightFollowers[i-1] = new WPI_TalonFX(rightMotorIDs[i]);
            }
        }
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

    /* CONTROL METHODS */
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

    // sets the speeds of the speedcontrollergroups rather than the differentrialdrive
    public void setDrive(double speed) {
        leftGroup.set(speed);
        rightGroup.set(speed);
    }

    /* PID AND ODOMETRY */
    // NOTE: many of the following methods are self-explanatory. Only the neccesary documentation will be made.
    public double getLeftPositionRotation() {
        return ticksToRotations(leftLeader.getSelectedSensorPosition());
    }

    public double getRightPositionRotation() {
        return ticksToRotations(-rightLeader.getSelectedSensorPosition());
    }

    private double ticksToRotations(double ticks) {
        return ticks / constants.epr / constants.gearing;
    }

    private double rotationsToMeters(double rotations) {
        // number of rotations * circumference of wheel
        return rotations * constants.wheelDiameterMeters * Math.PI;
    }

    public double getLeftPositionMeters() {
        return rotationsToMeters(getLeftPositionRotation());
    }

    public double getRightPositionMeters() {
        return rotationsToMeters(getRightPositionRotation());
    }

    public double getLeftVelocityRPS() {
        return ticksToRotations(leftLeader.getSelectedSensorVelocity()); // TODO: Removed / 2. Check if correct.
    }

    public double getRightVelocityRPS() {
        return ticksToRotations(rightLeader.getSelectedSensorVelocity()) ; // TODO: Removed / 2. Check if correct.
    }

    // converts rotations per second (RPS) to meters per second (MPS)
    private double rpsToMPS(double rotationsPerSecond) {
        return rotationsPerSecond * constants.wheelDiameterMeters * Math.PI;
    }

    public double getLeftVelocityMPS() {
        return rpsToMPS(getLeftVelocityRPS());
    }

    public double getRightVelocityMPS() {
        return rpsToMPS(getRightVelocityRPS());
    }

    // gets the average velocity of the robot by taking the mean of both sides
    public double getAbsoluteAverageVelocityMPS() {
        return (Math.abs(getRightVelocityMPS()) + Math.abs(getLeftVelocityMPS())) / 2;
    }

    // gets the direction of the robot
    public double getHeading() {
        try {
            return ahrs.getAngle() * (invertGyro ? -1 : 1);
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException" + e + "from getHeading. \n A constant was likely not given by DriveConstants object");
            return 0;
        }
    }

    public double getHeading360() {
        return Math.IEEEremainder(getHeading(), 360.0);
    }

    public double getHeading180() {
        double heading = getHeading() % 360;
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
        // System.out.println("1:  " + ahrs.getAngle());
        ahrs.reset();
        // System.out.println("2:  " + ahrs.getAngle());
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
        return ahrs.getRate(); // TODO: negative sign removed, check if correct
    }

    // sets the idle modes of all motors
    public void setNeutralModes(NeutralMode idleMode) {
        leftLeader.setNeutralMode(idleMode);
        rightLeader.setNeutralMode(idleMode);
        for(WPI_TalonFX motor : leftFollowers) {
            motor.setNeutralMode(idleMode);
        }
        for(WPI_TalonFX motor : rightFollowers) {
            motor.setNeutralMode(idleMode);
        }
    }

    // checks if the pidcontroller is at the last setpoint
    public boolean atSetpoint() {
        return turnController.atSetpoint();
    }

    // gets the last setpoint of the pidcontroller
    public double getLastAngleSetpoint() {
        return lastAngleSetpoint;
    }
    
    /* PIDSUBSYSTEMS OVERRIDES */
    // the next few methods override PIDSubsystem. 
    @Override
    public void periodic() {
        // Call periodic of PIDSubsystem to ensure PID controller runs
        super.periodic();

        // Update the pose
        Rotation2d gyroAngle = Rotation2d.fromDegrees(getHeading360());
        odometry.update(gyroAngle, getLeftPositionMeters(), getRightPositionMeters());
        // System.out.println(""+getLeftPositionMeters() +"  "+ getRightPositionMeters());

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
        try {
            arcadeDrive(turnFPIDThrottle, (output + Math.copySign(turnFPID.kF, output)));
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException" + e + "from useOutput. \n A constant was likely not given by DriveConstants object");
        }
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

    // stops all motors
    public void stop() {
        differentialDrive.stopMotor();
    }
}
