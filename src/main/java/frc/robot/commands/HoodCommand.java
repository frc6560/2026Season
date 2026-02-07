// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.ManualControls;

public class HoodCommand extends Command {
  
  private final Hood hood;
  private final ManualControls controls;
  private final ShotCalculator shotCalc;

  /** * Creates a new HoodCommand.
   * Order matches RobotContainer: (Hood, ManualControls, ShotCalculator)
   */
  public HoodCommand(Hood hood, ManualControls controls, ShotCalculator shotCalc) {
    this.hood = hood;
    this.controls = controls;
    this.shotCalc = shotCalc;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    // Optional: Reset or set initial state
  }

  @Override
  public void execute() {
    // 1. Check for Manual Overrides first
    if (controls.hoodManualUp()) {
      hood.manualUp();
    }
    else if (controls.hoodManualDown()) {
      hood.manualDown();
    }
    // 2. Check for Auto-Aim (Shooting)
    // You likely have a button binding for this in ManualControls
    else if (controls.shootWithLimelight()) {
      // Use the calculator to get the correct angle for the current distance
      hood.setGoalFromCalculator(shotCalc);
    }
    // 3. Otherwise, hold current position (automatic via internal PID)
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Default commands never finish
  }
}