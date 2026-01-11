package com.team6560.frc2026.commands;

import com.team6560.frc2026.controls.XboxControls;
import com.team6560.frc2026.subsystems.BallGrabber;

import edu.wpi.first.wpilibj2.command.Command;
public class BallGrabberCommand extends Command {
    
    final BallGrabber ballGrabber;
    final XboxControls controls;

    public BallGrabberCommand(BallGrabber grabber, XboxControls controls) {
        this.ballGrabber = grabber;
        this.controls = controls;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        ballGrabber.stop();
    }
    
    @Override
    public void execute() {
        ballGrabber.stop();
    }

    @Override
    public void end(boolean interrupted) {
        ballGrabber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


