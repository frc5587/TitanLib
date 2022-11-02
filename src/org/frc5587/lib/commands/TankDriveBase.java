package org.frc5587.lib.commands;

import org.frc5587.lib.subsystems.DrivetrainBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Tank Drive for a Differential Drive
 */
public class TankDriveBase extends CommandBase {
    private final DrivetrainBase drivetrain;
    private final DoubleSupplier leftThrottleSupplier, rightThrottleSupplier;

    // TankDrive uses input from two joysticks to drive each side of the robot.
    public TankDriveBase(DrivetrainBase drivetrain, DoubleSupplier leftThrottleSupplier, DoubleSupplier rightThrottleSupplier) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.leftThrottleSupplier = leftThrottleSupplier;
        this.rightThrottleSupplier = rightThrottleSupplier;
    }
    @Override
        public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var leftThrottle = leftThrottleSupplier.getAsDouble();
        var rightThrottle = rightThrottleSupplier.getAsDouble();
        drivetrain.tankDriveVolts(leftThrottle, rightThrottle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}