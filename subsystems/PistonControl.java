package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PistonControl extends SubsystemBase{
    private DoubleSolenoid[] doubleSolenoidPistons;

    public PistonControl(int[] pistonIDS) {
        for(int i = 0; i < pistonIDS.length; i+=2) {
            doubleSolenoidPistons[i] = new DoubleSolenoid(i, i+1);
        }

    }

    public void extend() {
        for(DoubleSolenoid piston : doubleSolenoidPistons) {
            piston.set(kForward);
        }
    }

    public void retract() {
        for(DoubleSolenoid piston : doubleSolenoidPistons) {
            piston.set(kReverse);
        }
    }
}
