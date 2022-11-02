package org.frc5587.lib.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frc5587.lib.subsystems.DrivetrainBase;

import java.util.function.DoubleSupplier;

/**
 * Arcade Drive for a DifferentialDrive
 */
public class ArcadeDrive extends CommandBase {
    private final DrivetrainBase drivetrain;
    private final DoubleSupplier throttleSupplier, curveSupplier;

    double throttle;
    double curve;

    public ArcadeDrive(DrivetrainBase drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier curveSupplier) {
        this.drivetrain = drivetrain;
        this.throttleSupplier = throttleSupplier;
        this.curveSupplier = curveSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        throttle = throttleSupplier.getAsDouble();
        curve = curveSupplier.getAsDouble();

        drivetrain.arcadeDrive(throttle, curve);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}