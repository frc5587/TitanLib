package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFPIDController;
import org.frc5587.lib.pid.FPID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class PivotingArmBase extends FPIDSubsystem {
    protected FFPIDController controller;
    
    public PivotingArmBase(FPIDConstants constants, SpeedController motorGroup) {
        super(constants, motorGroup);
        controller = constants.ff;
    }

    public double calcFeedForward(double position, double velocity, double acceleration) {
        return controller.calculateArm(position, velocity, acceleration);
    }

    public double calcFeedForward(double position) {
        return controller.calculateArm(position);
    }

    public double rotationsToMeasurement(double rotations) {
        return getPositionDegrees();
    }

    public double getPositionDegrees() {
        return getRotations() * 360;
    }

    @Override
    public void periodic() {
        double setpoint = 30;
        double output = getController().calculate(Math.toRadians(getMeasurement()), setpoint);
        // useOutput(output, setpoint);
        System.out.println("" + getMeasurement() + "      " + getRotations() + "         " + output);
        System.out.println("FF is" + calcFeedForward(Math.toRadians(getMeasurement())));
        useOutput(output + controller.calculateArm(Math.toRadians(getMeasurement())), setpoint);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        // System.out.println("output is" + output);
        set(output);
    }
    // protected ArmsConstants constants;
    // // the leader motor is defined as its own variable, while followers are put into an array for easy use.
    // protected SpeedController leader;
    // protected SpeedController[] followers;
    // protected SpeedControllerGroup motorGroup;

    // private DigitalInput armLimitSwitch;
    // private PIDController controller;

    // // what type of value we can get from an encoder
    // public static enum EncoderValueType {
    //     Velocity, Position
    // }

    // public static class ArmsConstants {
    //     public double armSpeedMultiplier;
    //     public int limitSwitchPort;
    //     public FPID fpid;
    //     public ArmFeedforward ff;

    //     public ArmsConstants(double armSpeedMultiplier, int limitSwitchPort, FPID fpid, ArmFeedforward ff) {
    //         this.armSpeedMultiplier = armSpeedMultiplier;
    //         this.limitSwitchPort = limitSwitchPort;
    //         this.fpid = fpid;
    //         this.ff = ff;
    //     }
    // }

    // // pass motors as an array of SpeedControllers, eg. WPI_TalonFX[] or CANSparkMax[]
    // public PivotingArmBase(ArmsConstants constants, SpeedController[] motors) {
    //     super(new PIDController(constants.fpid.kP, constants.fpid.kI, constants.fpid.kD));
    //     // set the controller variable to the ArmFPIDController we just made above
    //     controller = getController();
    //     //disable PID control when starting
    //     this.disable();

    //     /* THE BELOW LINES WILL BE USED AFTER CHARACTERIZATION. DO NOT USE THEM NOW */

    //     // this.enable();
    //     // controller.enableFeedForward();

    //     this.constants = constants;
    //     armLimitSwitch = new DigitalInput(constants.limitSwitchPort);

    //     // make followers a new array of SpeedControllers that is 1 value less long than the motors array (because the leader won't be used)
    //     followers = new SpeedController[motors.length-1];
    //     // put each motor in an array corresponding to its type (leader or follower)
    //     for(int i = 0; i < motors.length; i++) {
    //         // if it's the first one in the array, it's a leader
    //         if(i == 0) {
    //             // make a new motor with the given ID and put it in the array
    //             this.leader = motors[i];
    //         }
    //         // otherwise it's a follower
    //         else {
    //             followers[i-1] = motors[i];
    //         }
    //     }

    //     // if there are no followers, don't put the array in the speedcontroller group
    //     if(followers.length == 0) {
    //         motorGroup = new SpeedControllerGroup(leader);
    //     }
    //     else {
    //         motorGroup = new SpeedControllerGroup(leader, followers);
    //     }

    //     configureMotors();
    // }

    // // have the implementing class configure the motors
    // public abstract void configureMotors();

    // // the implementing class also needs to get and set encoder values
    // // param type: what type of value we're getting from the encoder. values Position and Velocity as defined above in EncoderValueType
    // public abstract double getEncoderValue(EncoderValueType type);

    // public abstract void setEncoderPosition(double position);

    // public PIDController getArmController() {
    //     return controller;
    // }

    // // move the arm based on a given throttle 
    // public void moveArmThrottle(double throttle) {
    //     motorGroup.set(-throttle * constants.armSpeedMultiplier);
    // }

    // // move the arm based on a constant multiplier (for operation with buttons)
    // public void moveArmFixedSpeed() {
    //     motorGroup.set(constants.armSpeedMultiplier);
    // }

    // // reverse the above function (for operation with buttons)
    // public void moveArmFixedReversed() {
    //     motorGroup.set(-constants.armSpeedMultiplier);
    // }

    // // moves the arm based on voltage instead of speed
    // public void moveArmVolts(double voltage) {
    //     motorGroup.setVoltage(voltage);
    // }

    // public void moveArmVoltsReversed(double voltage) {
    //     motorGroup.setVoltage(-voltage);
    // }

    // public void startPID() {
    //     SmartDashboard.putNumber("Goto Position", 0);
    // }

    // // displays various PID values on SmartDashboard
    // public void refreshPID() {
    //     SmartDashboard.putNumber("Angle", getPositionDegrees());
    //     SmartDashboard.putNumber("Encoder Val", getPositionRotation());
    // }
    
    // // gets the encoder's position
    // public double getPositionRotation() {
    //     return getEncoderValue(EncoderValueType.Position);
    // }

    // public double getPositionDegrees() {
    //     return getPositionRotation() / 180;
    // }

    // public DigitalInput getArmLimitSwitch() {
    //     return armLimitSwitch;
    // }

    // public boolean getLimitSwitchValue() {
    //     return !(armLimitSwitch.get());
    // }
    
    // // uses PID output to move the arm
    // @Override
    // protected void useOutput(double output, double setpoint) {
    //     try {
    //         moveArmThrottle(output);
    //     }
    //     catch(NullPointerException e) {
    //         System.out.println("NullPointerException " + e + " from useOutput. \n A constant was likely not given by DriveConstants object");
    //     }
    // }

    // @Override
    // protected double getMeasurement() {
    //     return getEncoderValue(EncoderValueType.Position);
    // }

    // @Override
    // public void periodic() {
    //     System.out.println(getPositionDegrees());
    //     refreshPID();
    //     // double setpoint = constants.ff.calculate(Math.toRadians(getPositionDegrees()), getEncoderValue(EncoderValueType.Velocity));
    //     // double output = controller.calculate(getMeasurement(), setpoint, getEncoderValue(EncoderValueType.Velocity));
    //     // useOutput(output, setpoint);
    // }

    // // sets encoders back to 0
    // public void resetEncoders() {
    //     setEncoderPosition(0);
    // }

    // // stops the speedcontroller group
    // public void stop() {
    //     motorGroup.set(0);
    // }

    // // stops every motor without going through the speedcontroller group
    // public void stopMotors() {
    //     leader.set(0);
    //     for(SpeedController follower : followers) {
    //         follower.set(0);
    //     }
    // }
}
