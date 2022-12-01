package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PistonControl extends SubsystemBase {
    private DoubleSolenoid[] doubleSolenoidPistons;

    /**
     * Creates new doublesolenoids for the amount of piston ports specified in
     * constants.java
     */
    public PistonControl(DoubleSolenoid[] solenoids) {
        doubleSolenoidPistons = solenoids;
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
