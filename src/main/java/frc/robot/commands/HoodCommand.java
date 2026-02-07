package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.ManualControls;

public class HoodCommand extends Command {
  
  private final Hood hood;
  private final ManualControls controls;
  private final ShotCalculator shotCalc;

  public HoodCommand(Hood hood, ManualControls controls, ShotCalculator shotCalc) {
    this.hood = hood;
    this.controls = controls;
    this.shotCalc = shotCalc;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    // SAFETY: Sync the software profile to the real hood angle.
    // This prevents the hood from snapping if it fell while disabled.
    hood.resetProfileToCurrent();
  }

  @Override
  public void execute() {
    // 1. Update the Goal (Where do we want to go?)
    if (controls.hoodManualUp()) {
      hood.manualUp();
    }
    else if (controls.hoodManualDown()) {
      hood.manualDown();
    }
    else if (controls.shootWithLimelight()) {
      // Auto-Aim: Ask calculator for the perfect angle
      hood.setGoalFromCalculator(shotCalc);
    }
    
    // 2. CRITICAL FIX: Actually run the control loop!
    // Without this line, the motor will never get a signal.
    hood.runControlLoop();
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}