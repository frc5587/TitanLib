package org.frc5587.lib.pathfinder;

import com.ctre.phoenix.motion.SetValueMotionProfile;

import edu.wpi.first.wpilibj.command.Command;

public class MotionProfileRunner extends Command {
    PhoenixAbstractDrive drive;
    boolean ready = false;

    public MotionProfileRunner(PhoenixAbstractDrive drivetrain) {
        this.drive = drivetrain;
        requires(drivetrain);
    }

    public void initialize() {
        drive.setProfileMode(SetValueMotionProfile.Disable);
        drive.enableBrakeMode(true);
    }

    public void execute() {
        if (!ready) {
            ready = drive.isMPReady();
        } else {
            drive.setProfileMode(SetValueMotionProfile.Enable);
        }
        drive.sendDebugInfo();
        drive.sendMPDebugInfo();
    }

    public boolean isFinished() {
        return drive.isMPDone();
    }

    public void end() {
        drive.resetMP();
        drive.stop();
        drive.enableBrakeMode(false);
    }

    public void interrupted() {
        end();
    }
}