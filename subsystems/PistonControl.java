package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PistonControl extends SubsystemBase {
    private DoubleSolenoid[] doubleSolenoidPistons;

/**
 * Creates new doublesolenoids for the amount of piston ports specified in
 * constants.java
 */
public PistonControl(int[][] pistonIDS) {
    doubleSolenoidPistons = new DoubleSolenoid[pistonIDS.length];

    for (int i = 0; i < pistonIDS.length; i++) {
        doubleSolenoidPistons[i] = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pistonIDS[i][0], pistonIDS[i][1]);
    }
}

// Implementation: pistonsSet(kForward)
public void pistonsSet(Value direction) {
    for (DoubleSolenoid piston : doubleSolenoidPistons) {
        piston.set(direction);
    }
}

// Utilize these 2 methods as alternatives to the pistonsSet method
public void extend() {
    pistonsSet(Value.kForward);
}

public void retract() {
    pistonsSet(Value.kReverse);
}

}
