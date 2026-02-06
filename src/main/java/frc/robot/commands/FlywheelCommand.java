// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.ManualControls;

/**
 * Default command for the flywheel.
 * Uses ShotCalculator to determine the appropriate RPM based on distance to target.
 */
public class FlywheelCommand extends Command {

  private final Flywheel flywheel; 
  private final ShotCalculator shotCalculator;
  private final ManualControls controls;

  /**
   * Creates a new FlywheelCommand.
   * @param flywheel The flywheel subsystem
   * @param shotCalculator The shot calculator for determining RPM
   * @param controls Manual controls (for future manual override)
   */
  public FlywheelCommand(Flywheel flywheel, ShotCalculator shotCalculator, ManualControls controls) {
    this.flywheel = flywheel;
    this.shotCalculator = shotCalculator;
    this.controls = controls;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Optionally set to idle on start
    // flywheel.setIdle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Use ShotCalculator to set RPM based on distance
    flywheel.setRPMFromCalculator(shotCalculator);
    
    // Alternative: Set to idle if no valid shot
    // if (shotCalculator.hasValidShot()) {
    //   flywheel.setRPMFromCalculator(shotCalculator);
    // } else {
    //   flywheel.setIdle();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Keep spinning at idle or stop completely
    flywheel.setIdle(); // Keep at idle speed
    // Or use: flywheel.stop(); // Stop completely
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This is a default command - runs continuously
    return false;
  }
}