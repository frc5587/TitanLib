package org.frc5587.lib.pathfinder;

import org.frc5587.lib.pid.PIDVA;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class GyroCompMPRunner extends CommandBase {
    private AbstractDrive drive;
    private EncoderFollower lEncoderFollower, rEncoderFollower;
    private Notifier looper;
    private String name = "Anonymous Trajectory";
    private boolean forwards, pathFinished = false;
    private PIDVA pidvaLeft, pidvaRight;
    private double wheelDiameterMeters, initialHeading = 0;

    public GyroCompMPRunner(AbstractDrive drive, String pathName, Pathgen pathgen, PIDVA pidvaLeft, PIDVA pidvaRight,
            double gyrokP) {
        this(drive, Pathgen.getTrajectoryFromFile(pathName), true, pathgen, pidvaLeft, pidvaRight, gyrokP);
        name = pathName;
    }

    public GyroCompMPRunner(AbstractDrive drive, String pathName, boolean forwards, Pathgen pathgen, PIDVA pidvaLeft,
            PIDVA pidvaRight, double gyrokP) {
        this(drive, Pathgen.getTrajectoryFromFile(pathName), forwards, pathgen, pidvaLeft, pidvaRight, gyrokP);
        name = pathName;
    }

    public GyroCompMPRunner(AbstractDrive drive, Trajectory trajectory, Pathgen pathgen, PIDVA pidvaLeft,
            PIDVA pidvaRight, double gyrokP) {
        this(drive, trajectory, true, pathgen, pidvaLeft, pidvaRight, gyrokP);
    }

    public GyroCompMPRunner(AbstractDrive drive, Trajectory trajectory, boolean forwards, Pathgen pathgen,
            PIDVA pidvaLeft, PIDVA pidvaRight, double gyrokP) {
        this(drive, pathgen.getLeftSide(trajectory), pathgen.getRightSide(trajectory), forwards, pidvaLeft, pidvaRight,
                gyrokP);
        System.out.println("Trajectory length: " + trajectory.length());
    }

    public GyroCompMPRunner(AbstractDrive drive, Trajectory leftTraj, Trajectory rightTraj, PIDVA pidvaLeft,
            PIDVA pidvaRight, double gyrokP) {
        this(drive, leftTraj, rightTraj, true, pidvaLeft, pidvaRight, gyrokP);
    }

    public GyroCompMPRunner(AbstractDrive drive, Trajectory leftTraj, Trajectory rightTraj, boolean forwards,
            PIDVA pidvaLeft, PIDVA pidvaRight, double gyrokP) {
        addRequirements(drive);

        this.drive = drive;
        this.forwards = forwards;

        this.pidvaLeft = pidvaLeft;
        this.pidvaRight = pidvaRight;

        this.wheelDiameterMeters = drive.wheelDiameterMeters;

        this.lEncoderFollower = new EncoderFollower(leftTraj);
        this.rEncoderFollower = new EncoderFollower(rightTraj);
        this.looper = new Notifier(new ProfileLooper(drive.stuPerRev, gyrokP));
    }

    @Override
    public String toString() {
        return this.getClass().getName() + ": " + name;
    }

    public void initialize() {
        lEncoderFollower.configureEncoder(drive.getLeftPosition(), drive.stuPerRev, wheelDiameterMeters);
        rEncoderFollower.configureEncoder(drive.getRightPosition(), drive.stuPerRev, wheelDiameterMeters);

        lEncoderFollower.configurePIDVA(pidvaLeft.kP, pidvaLeft.kI, pidvaLeft.kD, pidvaLeft.kV, pidvaLeft.kA);
        rEncoderFollower.configurePIDVA(pidvaRight.kP, pidvaRight.kI, pidvaRight.kD, pidvaRight.kV, pidvaRight.kA);

        initialHeading = drive.getHeading();

        drive.enableBrakeMode(true);

        looper.startPeriodic(0.01);
    }

    public void execute() {
        drive.sendDebugInfo();
    }

    public boolean isFinished() {
        return pathFinished;
    }

    public void end() {
        System.out.println("GyroCompMPRunner: " + name + " ending");
        looper.stop();
        drive.stop();
        pathFinished = true;
    }

    public void interrupted() {
        System.out.println("GyroCompMPRunner: " + name + " interrupted.");
        looper.stop();
        drive.stop();
        pathFinished = true;
    }

    public class ProfileLooper implements Runnable {
        private int stuPerInch;
        private double gyrokP;

        public ProfileLooper(int stuPerInch, double gyrokP) {
            this.stuPerInch = stuPerInch;
            this.gyrokP = gyrokP;
        }

        @Override
        public void run() {
            if (!pathFinished) {
                double left = 0, right = 0;

                if (!lEncoderFollower.isFinished()) {
                    SmartDashboard.putNumber("Left Expected Pos", lEncoderFollower.getSegment().position * stuPerInch);
                    SmartDashboard.putNumber("Left Expected Vel",
                            lEncoderFollower.getSegment().velocity * stuPerInch / 10f);
                    SmartDashboard.putNumber("Left Pos Error",
                            lEncoderFollower.getSegment().position - drive.getLeftPosition() / (double) stuPerInch);
                    SmartDashboard.putNumber("Left Vel Error", lEncoderFollower.getSegment().velocity
                            - drive.getLeftVelocity() / (double) stuPerInch * 10);
                }
                if (!rEncoderFollower.isFinished()) {
                    SmartDashboard.putNumber("Right Expected Pos", rEncoderFollower.getSegment().position * stuPerInch);
                    SmartDashboard.putNumber("Right Expected Vel",
                            rEncoderFollower.getSegment().velocity * stuPerInch / 10f);
                    SmartDashboard.putNumber("Right Pos Error",
                            rEncoderFollower.getSegment().position - drive.getRightPosition() / (double) stuPerInch);
                    SmartDashboard.putNumber("Left Vel Error", rEncoderFollower.getSegment().velocity
                            - drive.getRightVelocity() / (double) stuPerInch * 10);
                }

                if (forwards) {
                    left = lEncoderFollower.calculate(drive.getLeftPosition());
                    right = rEncoderFollower.calculate(drive.getRightPosition());
                } else {
                    left = -rEncoderFollower.calculate(-drive.getLeftPosition());
                    right = -lEncoderFollower.calculate(-drive.getRightPosition());
                }

                double gyroHeading = drive.getHeading();
                double desiredHeading = Pathfinder.r2d(lEncoderFollower.getHeading());
                double angleDifference = Pathfinder.boundHalfDegrees(initialHeading + desiredHeading - gyroHeading);

                SmartDashboard.putNumber("Gyro Heading", gyroHeading);
                SmartDashboard.putNumber("Desired Heading", desiredHeading);
                SmartDashboard.putNumber("Angle Difference", angleDifference);

                double turn = gyrokP * angleDifference;

                left += turn;
                right -= turn;

                // System.out.println("Left: " + left + " Right: " + right );

                pathFinished = lEncoderFollower.isFinished() && rEncoderFollower.isFinished();
                drive.vbusLR(left, right);
            }
        }
    }
}