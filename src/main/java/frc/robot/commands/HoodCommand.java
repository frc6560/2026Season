package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.Hood; // Adjust if in superstructure
import frc.robot.subsystems.ShotCalculator; // Adjust if in superstructure

public class HoodCommand extends Command {
  
  // FIX: Renamed from 'Hood' to 'hood' (lowercase)
  private final Hood hood; 
  private final ManualControls controls;
  private final ShotCalculator shotCalculator;

  public HoodCommand(Hood hood, ShotCalculator shotCalculator, ManualControls controls) {
    this.hood = hood; 
    this.shotCalculator = shotCalculator;
    this.controls = controls;
    addRequirements(hood);
  }

  @Override
  public void initialize() {}
  public void execute() {
    if (controls.shootWithLimelight()) {
      hood.setGoalFromCalculator(shotCalculator); 
    }
    else if (controls.hoodManualUp()) { 
      hood.manualUp();
    }
    else if (controls.hoodManualDown()) {
      hood.manualDown();
    }
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