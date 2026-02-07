// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

    // Gear ratio: 24:18 (motor:flywheel)
    private static final double GEAR_RATIO = 24.0 / 18.0;

    // PID constants
    private static final double kP = 0.11;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    private static final double kS = 0.0;

    private double targetRPM = 0;

    public FLywheel() {
        leaderMotor = new TalonFX(20, "rio");
        followerMotor = new TalonFX(21, "rio");
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;

        leaderMotor.getConfigurator().apply(config);

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = 80;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = 40;

        followerMotor.getConfigurator().apply(followerConfig);

        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0 * GEAR_RATIO;
        leaderMotor.setControl(velocityControl.withVelocity(rps));
    }

    public void setPercent(double percent) {
        leaderMotor.set(percent);
    }

    public void stop() {
        targetRPM = 0;
        leaderMotor.stopMotor();
    }

    public double getVelocityRPM() {
        double motorRPS = leaderMotor.getVelocity().getValueAsDouble();
        return motorRPS * 60.0 / GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getVelocityRPM() - targetRPM) < 100;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/Actual RPM", getVelocityRPM());
        SmartDashboard.putNumber("Flywheel/Leader Output", leaderMotor.get());
        SmartDashboard.putNumber("Flywheel/Follower Output", followerMotor.get());
        SmartDashboard.putBoolean("Flywheel/At Setpoint", atSetpoint());
    }
}



package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
  // PID controller for flywheel (software fallback)
  private final PIDController pidController;



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

    pidController = new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);

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
    double motorRPM = leftFlywheelMotor.getRotorVelocity().getValueAsDouble(); 
    return motorRPM / FlywheelConstants.FLYWHEEL_GEAR_RATIO;
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
    // Telemetry for AdvantageScope and SmartDashboard
    double currentRPM = getCurrentRPM();
    SmartDashboard.putNumber("Flywheel RPM", currentRPM);
    // Compute PID output (percent) using RPM error, pidController tuned in constructor
    double output = pidController.calculate(currentRPM, targetRPM);
    // clamp output
    leftFlywheelMotor.setControl(velocityControl.withVelocity(targetRPM / 60.0));  // Velocity control
    rightFlywheelMotor.setControl(velocityControl.withVelocity(targetRPM / 60.0));
    

    SmartDashboard.putNumber("Flywheel/Current RPM", currentRPM);
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
    SmartDashboard.putBoolean("Flywheel/At Target", atTargetRPM());
    SmartDashboard.putNumber("Flywheel/PID Output", output);
    SmartDashboard.putNumber("Flywheel/Voltage", leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Current Draw", leftFlywheelMotor.getSupplyCurrent().getValueAsDouble());

}
}