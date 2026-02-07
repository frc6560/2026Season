// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel; // Check this import path!
import frc.robot.subsystems.ShotCalculator;
import frc.robot.ManualControls;

public class FlywheelCommand extends Command {

  private final Flywheel flywheel; 
  private final ShotCalculator shotCalculator;
  private final ManualControls controls;

  /** * Creates a new FlywheelCommand. 
   * Updated to accept ShotCalculator to match RobotContainer.
   */
  public FlywheelCommand(Flywheel flywheel, ShotCalculator shotCalculator, ManualControls controls) {
    this.flywheel = flywheel;
    this.shotCalculator = shotCalculator;
    this.controls = controls;
    addRequirements(flywheel);
  }

  // Called when command starts (e.g., when robot enables)
  @Override
  public void initialize() {
    // Start at Idle speed immediately
    flywheel.setIdle();
  }

  @Override
  public void execute() {
    // 1. Check if the driver is holding the shoot button
    if (controls.shootWithLimelight()) {
        // BUTTON PRESSED: Spin up to high speed based on distance
        flywheel.setRPMFromCalculator(shotCalculator);
    } 
    else {
        // BUTTON RELEASED: Drop to Idle speed (e.g. 1000 RPM)
        flywheel.setIdle();
    }
  }

  // Called when command is interrupted (e.g. robot disabled)
  @Override
  public void end(boolean interrupted) {
    // Safety: Stop motors completely when disabled
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Default command never finishes
  }
}