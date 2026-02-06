// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.ManualControls;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodCommand extends Command {
  private final Hood Hood;
  private final ManualControls controls;
  private final double targetAngle;
  private final ShotCalculator shotCalculator;
  /** Creates a new HoodCommand. */
  public HoodCommand(Hood hood, ShotCalculator shotCalculator, ManualControls controls) {
    this.hood = hood;
    this.shotCalculator = shotCalculator;
    this.controls = controls;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setGoal(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.shootWithLimelight()) {
      Hood.setGoalFromCalculator(shotCalculator); 
    }
    else if (controls.hoodManualUp()) {
      Hood.manualUp();
    }
    else if (controls.hoodManualDown()) {
      Hood.manualDown();
    }
    else {
      
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
