package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PistonControl extends SubsystemBase{
    private DoubleSolenoid[] doubleSolenoidPistons;

    public PistonControl(int[][] pistonIDS) {
        doubleSolenoidPistons = new DoubleSolenoid[pistonIDS.length/2];

        for(int i = 0; i < pistonIDS.length; i++) {
            doubleSolenoidPistons[i] = new DoubleSolenoid(pistonIDS[i][0], pistonIDS[i][1]);
        }
    }

    public void pistonsSet(Value direction) {
        for(DoubleSolenoid piston : doubleSolenoidPistons) {
            piston.set(direction);
        }
    }

    public void extend() {
        for(DoubleSolenoid piston : doubleSolenoidPistons) {
            piston.set(Value.kForward);
        }
    }

    public void retract() {
        for(DoubleSolenoid piston : doubleSolenoidPistons) {
            piston.set(Value.kReverse);
        }
    }
}
