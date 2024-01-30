package org.frc5587.lib.control;

import org.frc5587.lib.subsystems.DifferentialDriveBase;
import org.frc5587.lib.subsystems.DifferentialDriveBase.FollowDirection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TitanDrive extends CommandBase {
    private final DifferentialDriveBase drivetrain;

    private final DeadbandCommandJoystick joystick;
    private final int turnModeButtonID = 1;
    private final int followModeButtonID = 3;
    private final int resetHeadingButtonID = 4;
    private final ControlMode controlMode;
    private FollowDirection currentMode = FollowDirection.AUTO;

    public TitanDrive(DifferentialDriveBase drivetrain, DeadbandCommandJoystick joystick, ControlMode controlMode) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.controlMode = controlMode;

        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        Translation2d joystickPosition = joystick.getPosition();
        Rotation2d desiredHeading = (new Rotation2d(joystickPosition.getY(), joystickPosition.getX())).plus(Rotation2d.fromDegrees(180));

        if (joystick.button(resetHeadingButtonID).getAsBoolean()) {
            drivetrain.zeroHeading();
        }

        switch (controlMode) {
            case HoldFlip:
                if (joystick.button(followModeButtonID).getAsBoolean()) {
                    if (currentMode == FollowDirection.AUTO) {
                        currentMode = drivetrain.getAutoFollowDirection(desiredHeading).opposite();
                    } else {
                        currentMode = currentMode.opposite();
                    }
                } else if (joystick.button(followModeButtonID).getAsBoolean()) {
                    currentMode = FollowDirection.AUTO;
                }
                break;

            case HoldForward:
                if (joystick.button(followModeButtonID).getAsBoolean()) {
                    currentMode = FollowDirection.FORWARD;
                } else {
                    currentMode = FollowDirection.AUTO;
                }
                break;

            case Manual:
                if (joystick.button(followModeButtonID).getAsBoolean()) {
                    currentMode = FollowDirection.BACKWARD;
                } else {
                    currentMode = FollowDirection.FORWARD;
                }
                break;

            case Toggle:
                if (joystick.button(followModeButtonID).getAsBoolean()) {
                    currentMode = drivetrain.getAutoFollowDirection(desiredHeading).opposite();
                } else if (currentMode == drivetrain.getAutoFollowDirection(desiredHeading)) {
                    currentMode = FollowDirection.AUTO;
                }
                break;
            default:
                throw new RuntimeException("controlMode set to: " + controlMode);
        }



        drivetrain.titanDrive(desiredHeading, joystickPosition.getNorm(), currentMode, joystick.button(turnModeButtonID).getAsBoolean());
    }
    
    public enum ControlMode {
        HoldForward,  // holding button forces robot to go forward, otherwise it will go whichever is closer
        Manual,       // holding button forces robot to go backward, otherwise it will go backward
        HoldFlip,     // holding button forces robot to go the opposite way, otherwise it will go whichever is closer
        Toggle;       // pressing button forces robot to flip which directions it following in

    } 
}