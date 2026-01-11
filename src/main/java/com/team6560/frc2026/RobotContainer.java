package com.team6560.frc2026;

import com.team6560.frc2026.Constants.ElevatorConstants;
import com.team6560.frc2026.Constants.OperatorConstants;
import com.team6560.frc2026.Constants.WristConstants;

import com.team6560.frc2026.controls.XboxControls;
import com.team6560.frc2026.utility.Enums.ReefLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.util.Set;

// took out subsystems + added camera
public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final XboxController firstXbox = new XboxController(0);
  final XboxController secondXbox = new XboxController(1);

  private final XboxControls controls = new XboxControls(firstXbox, secondXbox);
 



  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() { 
    
    driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
  }

  public void elevL4BeginTele() { // values for auto (don't touch!)
  }

  public void resetLLBeforeAuto() {
  }
  
  // don't randomly brake/unbrake chassis
  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }

  public XboxControls getControls() {
    return controls;
  }
}