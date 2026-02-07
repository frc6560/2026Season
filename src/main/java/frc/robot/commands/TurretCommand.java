package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.Turret;

/**
 * Default command for the turret. Watches the ManualControls buttons and sets
 * turret goals. Also exposes a simple manual percent output when no button is
 * pressed (useful for quick hardware testing).
 */
public class TurretCommand extends Command {
    private final Turret turret;
    private final ManualControls controls;

    public TurretCommand(Turret turret, ManualControls controls) {
        this.turret = turret;
        this.controls = controls;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // Buttons take precedence: set discrete goals
        if (controls.Turret90()) {
            turret.setGoal(90);
        } else if (controls.Turret0()){
            turret.setGoal(0);
        } else {
            // No button -> allow small manual open-loop from the second controller's right X axis
            // This is helpful for testing if closed-loop motion isn't responsive.
            double manual = controls.getTurretManual();
            if (Math.abs(manual) > 0.05) {
                turret.setPercent(manual * 0.4); // scale down for safety
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopMotor();
    }
}