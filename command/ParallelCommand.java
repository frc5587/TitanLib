package org.frc5587.lib.command;

import edu.wpi.first.wpilibj.command.Command;

public class ParallelCommand extends Command {
    Command[] commands;

    /**
     * This is a class used to run two or more commands simultaneously in a more
     * intuitive manner than with stock WPILib. Rather than relying on
     * CommandGroups for simply running a small number of commands in parallel and
     * the often confusing addParallel method in CommandGroups, one can simply use
     * this class in place of any normal command.
     */
    public ParallelCommand(Command... commands) {
        this.commands = commands;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        for (Command command : commands) {
            command.start();
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // Check if any commands are not complete and return false if they aren't
        for (Command command : commands) {
            if (!command.isCompleted()) {
                return false;
            }
        }
        // If all are complete, return true
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        for (Command command : commands) {
            command.cancel();
        }
    }
}