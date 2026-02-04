// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;



public class Flywheel extends SubsystemBase {

  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;

  //control 
  private final VelocityVoltage velocityControl;

  private final NetworkTable limelightTable; 

  //pose supplier

    // get pose from swerve subsystem

  //state
  private double targetRPM = 0.0;



  /** Creates a new Flywheel. */
  public Flywheel() {

    // initialize limelight network table
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //initialize motor
    leftFlywheelMotor = new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_ID, "rio");
    rightFlywheelMotor = new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_ID, "rio");

    configureMotor(leftFlywheelMotor, true); 
    configureMotor (rightFlywheelMotor, false);

    //initialize control
    velocityControl = new VelocityVoltage(0.0).withSlot(0);

  }

    private void configureMotor(TalonFX flywheelMotor, boolean inverted){ 
    //motor configuration
    TalonFXConfiguration config = new TalonFXConfiguration();

    //pid config
    //config.Slot0 = FlywheelConstants.FLYWHEEL_PID_CONFIG;
    config.Slot0.kP = FlywheelConstants.kP;
    config.Slot0.kI = FlywheelConstants.kI;
    config.Slot0.kD = FlywheelConstants.kD;
    config.Slot0.kV = FlywheelConstants.kV;
    //config.Slot0 = config.Slot0;

    //motor output 
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply inversion
    if (inverted) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }


    //current limits
    config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelMotor.getConfigurator().apply(config);
    }

    /**
   * Gets goal position based on alliance 
   * @return Goal position on field (meters)
   */

  public void setIdle(){
    setRPM(FlywheelConstants.FLYWHEEL_IDLE_RPM);
  }

  public void setRPM(double rpm){
    targetRPM = rpm;
    // actual motor output will be computed in periodic using PID
  }
  
  public void stop(){
    targetRPM = 0.0;
      leftFlywheelMotor.stopMotor();
      rightFlywheelMotor.stopMotor();
  }

  public double getCurrentRPM(){
    // Average both motors for robustness
    double leftVel = leftFlywheelMotor.getRotorVelocity().getValueAsDouble();
    double rightVel = rightFlywheelMotor.getRotorVelocity().getValueAsDouble();
    double avgMotorRPS = (Math.abs(leftVel) + Math.abs(rightVel)) / 2.0;

    // Convert motor RPS to flywheel RPM
    return (avgMotorRPS * 60.0) / FlywheelConstants.FLYWHEEL_GEAR_RATIO;
  }

  public double getTargetRPM(){
    return targetRPM;
  }

  public boolean atTargetRPM(){
    double currentRPM = getCurrentRPM();
    return Math.abs(currentRPM - targetRPM) < FlywheelConstants.FLYWHEEL_RPM_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentRPM = getCurrentRPM();

    // Safety: stop motors if target is zero
    if (Math.abs(targetRPM) < 1e-3) {
      leftFlywheelMotor.stopMotor();
      rightFlywheelMotor.stopMotor();
    } else {
      // Convert RPM to motor shaft rotations per second (accounting for gear ratio)
      double targetMotorRPS = (targetRPM * FlywheelConstants.FLYWHEEL_GEAR_RATIO) / 60.0;

      // Use hardware velocity control (runs at 1kHz onboard)
      leftFlywheelMotor.setControl(velocityControl.withVelocity(targetMotorRPS));
      rightFlywheelMotor.setControl(velocityControl.withVelocity(targetMotorRPS));
    }

    // Telemetry
    SmartDashboard.putNumber("Flywheel/Current RPM", currentRPM);
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
    SmartDashboard.putBoolean("Flywheel/At Target", atTargetRPM());
    SmartDashboard.putNumber("Flywheel/Voltage", leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Current Draw", leftFlywheelMotor.getSupplyCurrent().getValueAsDouble());

}
}