package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.RevolverSubsystem;

/**
 * Runs the revolver using its state machine.
 * While this command is active, it requests feeding.
 * When released or interrupted, it stops.
 */
public class RunRevolverCommand extends Command {

    private final RevolverSubsystem revolver;

    public RunRevolverCommand(RevolverSubsystem revolver) {
        this.revolver = revolver;
        addRequirements(revolver);
    }

    @Override
    public void initialize() {
        // Request the revolver to start feeding (state machine handles motors)
        revolver.requestFeed();
    }

    @Override
    public void execute() {
        // Nothing extra needed; state machine handles pan/pusher
    }

    @Override
    public void end(boolean interrupted) {
        // Stop feeding when command ends
        revolver.requestStop();
    }

    @Override
    public boolean isFinished() {
        // Runs while button is held
        return false;
    }
}



