package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PistonControl extends SubsystemBase{
    private DoubleSolenoid[] doubleSolenoidPistons;

    public PistonControl(int[] pistonIDS) {
        for(int i = 0; i < pistonIDS.length; i+=2) {  // for the amount of numbers in the pistonIDS array defined in constants
            doubleSolenoidPistons[i] = new DoubleSolenoid(i, i+1);  // create a new double solenoid
        }

    }

    public void extend() {  // method to extend the piston(s)
        for(DoubleSolenoid piston : doubleSolenoidPistons) { // extends the piston for every piston created
            piston.set(kForward);
        }
    }

    public void retract() {  // method to retract the piston(s)
        for(DoubleSolenoid piston : doubleSolenoidPistons) { // retracts the piston for every piston created
            piston.set(kReverse);
        }
    }
}
