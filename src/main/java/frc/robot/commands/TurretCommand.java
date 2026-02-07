package frc.robot.commands;

import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.Turret;

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
        if (controls.Turret90()) {
            turret.setGoal(90);
        } else if (controls.Turret0()){
            turret.setGoal(0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false; // This is a default command, so it never finishes
    }
}