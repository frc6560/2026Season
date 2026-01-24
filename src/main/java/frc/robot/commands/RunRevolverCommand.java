package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.RevolverSubsystem;

/**
 * Spins up pan first, then feeds until beam breaker sees a ball.
 * Motors idle at 10 RPM when command ends.
 */
public class RunRevolverCommand extends Command {

    private final RevolverSubsystem revolver;
    private boolean pusherStarted;

    public RunRevolverCommand(RevolverSubsystem revolver) {
        this.revolver = revolver;
        addRequirements(revolver);
    }

    @Override
    public void initialize() {
        pusherStarted = false;

        // Always start pan
        revolver.startPan();
    }

    @Override
    public void execute() {

        // If we already have a ball, do NOT push
        if (revolver.hasBall()) {
            return;
        }

        // Start pusher once pan is at speed
        if (revolver.panAtSpeed() && !pusherStarted) {
            revolver.startPusher();
            pusherStarted = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Return both motors to idle (10 RPM)
        revolver.stopBoth();
    }

    @Override
    public boolean isFinished() {
        // Command runs while button is held
        return false;
    }
}

