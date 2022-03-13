package org.frc5587.lib.control;

import org.frc5587.lib.MathHelper;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DeadbandXboxController extends XboxController {
    private static double DEFAULT_DEADBAND = 0.1;
    private final double deadbandCutoff;

    public JoystickButton aButton;
    public JoystickButton bButton;
    public JoystickButton xButton;
    public JoystickButton yButton;
    public JoystickButton leftBumper;
    public JoystickButton rightBumper;
    public POVButton dPadUp;
    public POVButton dPadDown;
    public POVButton dPadLeft;
    public POVButton dPadRight;
    public Trigger leftXTrigger;
    public Trigger leftYTrigger;
    public Trigger rightXTrigger;
    public Trigger rightYTrigger;
    public Trigger leftTrigger;
    public Trigger rightTrigger;

    /**
     * Construct an instance of an Xbox controller which ignores axis inputs that
     * are below the specified deadband cutoff.
     *
     * @param port           The port on the Driver Station that the joystick is
     *                       plugged into.
     * @param deadbandCutoff amount of deadband to apply to each axis
     */
    public DeadbandXboxController(int port, double deadbandCutoff) {
        super(port);
        this.deadbandCutoff = deadbandCutoff;
        instantiateButtons();
    }

    /**
     * Construct an instance of an Xbox controller which ignores axis inputs that
     * are below a default cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandXboxController(int port) {
        this(port, DEFAULT_DEADBAND);
    }

    @Override
    public double getRawAxis(int axis) {
        // Apply deadband when getting axis so all axes will have deadbanded signal
        return MathHelper.deadband(super.getRawAxis(axis), deadbandCutoff, 1);
    }

        /**
     * Whether the trigger is currently depressed.
     * 
     * @return the state of the trigger.
     */
    public boolean getLeftTrigger() {
        return super.getLeftTriggerAxis() > deadbandCutoff;
    }

    public boolean getRightTrigger() {
        return super.getRightTriggerAxis() > deadbandCutoff;
    }

    public void instantiateButtons() {
        aButton = new JoystickButton(this, DeadbandXboxController.Button.kA.value);
        bButton = new JoystickButton(this, DeadbandXboxController.Button.kB.value);
        xButton = new JoystickButton(this, DeadbandXboxController.Button.kX.value);
        yButton = new JoystickButton(this, DeadbandXboxController.Button.kY.value);
        leftBumper = new JoystickButton(this, DeadbandXboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(this, DeadbandXboxController.Button.kRightBumper.value);
        dPadUp = new POVButton(this, 0);
        dPadDown = new POVButton(this, 180);
        dPadLeft = new POVButton(this, 270);
        dPadRight = new POVButton(this, 90);
        leftXTrigger = new Trigger(() -> {return this.getLeftX() != 0;});
        leftYTrigger = new Trigger(() -> {return this.getLeftY() != 0;});
        rightXTrigger = new Trigger(() -> {return this.getLeftX() != 0;});
        rightYTrigger = new Trigger(() -> {return this.getLeftY() != 0;});
        leftTrigger = new Trigger(this::getLeftTrigger);
        rightTrigger = new Trigger(this::getLeftTrigger);
    }
}