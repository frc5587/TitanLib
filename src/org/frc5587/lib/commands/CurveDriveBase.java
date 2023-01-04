package org.frc5587.lib.commands;

import org.frc5587.lib.subsystems.DifferentialDriveBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Curve Drive for a DifferentialDrive
 */
public class CurveDriveBase extends CommandBase {
    private final DifferentialDriveBase drivetrain;
    private final double quickTurnCurveMultiplier;
    private final DoubleSupplier throttleSupplier, curveSupplier;
    private final BooleanSupplier quickTurnSupplier;

    public CurveDriveBase(DifferentialDriveBase drivetrain, double quickTurnCurveMultiplier, DoubleSupplier throttleSupplier, DoubleSupplier curveSupplier, BooleanSupplier quickTurnSupplier) {
        this.drivetrain = drivetrain;
        this.quickTurnCurveMultiplier = quickTurnCurveMultiplier;
        this.throttleSupplier = throttleSupplier;
        this.curveSupplier = curveSupplier;
        this.quickTurnSupplier = quickTurnSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double throttle = throttleSupplier.getAsDouble();
        double curve = curveSupplier.getAsDouble();
        boolean quickTurn = quickTurnSupplier.getAsBoolean();

        curve *= quickTurn ? quickTurnCurveMultiplier : 1;
        drivetrain.curvatureDrive(throttle, curve, quickTurn);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}