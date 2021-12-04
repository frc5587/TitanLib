package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PistonControl extends SubsystemBase{
    private DoubleSolenoid[] doubleSolenoidPistons;

    public PistonControl(int[] pistonIDS) {
        doubleSolenoidPistons = new DoubleSolenoid[pistonIDS.length/2];
        int[][] a= {
            {}
        };
        // TODO Create 2D array to rid of i+=2
        for(int i = 0; i < pistonIDS.length; i+=2) {
            doubleSolenoidPistons[i] = new DoubleSolenoid(pistonIDS[i], pistonIDs[i + 1]);
        }
    }

    // TODO Include extend() and retract() methods as an alternative
    public void pistonsSet(Value direction) {
        for(DoubleSolenoid piston : doubleSolenoidPistons) {
            piston.set(direction);
        }        
    }
}