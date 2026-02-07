// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  // Hardware
  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  // Mechanism2d visualization
  private final Mechanism2d hoodMech;
  private final MechanismRoot2d hoodRoot;
  private final MechanismLigament2d hoodArm;

  // Control
  private final PositionVoltage positionControl;

  // Trapezoidal motion profiling
  private final TrapezoidProfile.Constraints hoodConstraints;
  private final TrapezoidProfile hoodTrapezoidProfile;
  private TrapezoidProfile.State hoodGoalState;
  private TrapezoidProfile.State hoodSetpointState;

  // Target tracking
  private double targetAngle = 0.0;

  // Simulation
  private boolean isSimulation;
  private double simPosition = 0.0; // rotations
  private double simVelocity = 0.0; // rps

  // Get pose from swerve subsystem
  public interface PoseSupplier {
    Pose2d getPose();
  }

  /** Creates a new Hood. */
  public Hood(PoseSupplier poseSupplier) {
    this.isSimulation = RobotBase.isSimulation();

    // Initialize hardware (Removed "rio" to fix deprecation warnings)
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID);

    // Position control request
    positionControl = new PositionVoltage(0.0).withSlot(0);

    // Initialize trapezoidal profile
    hoodConstraints = new TrapezoidProfile.Constraints(
      HoodConstants.kMaxV,
      HoodConstants.kMaxA
    );
    hoodTrapezoidProfile = new TrapezoidProfile(hoodConstraints);
    hoodGoalState = new TrapezoidProfile.State();
    hoodSetpointState = new TrapezoidProfile.State();

    // Configure hardware
    configureAbsoluteEncoder();
    configureMotor();

    // Initialize Visualization
    hoodMech = new Mechanism2d(200, 200);
    hoodRoot = hoodMech.getRoot("Hood Root", 100, 100);
    hoodArm = hoodRoot.append(new MechanismLigament2d("Hood Arm", 50, 0));
    hoodArm.setColor(new Color8Bit(255, 165, 0));
    SmartDashboard.putData("Hood Mechanism", hoodMech);

    if (isSimulation) {
      System.out.println("Hood initialized in SIMULATION mode");
    }
  }

  /**
   * Configures the absolute encoder (CANcoder)
   */
  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    
    // Set sensor direction
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0; // Will be set during calibration
    
    absoluteEncoder.getConfigurator().apply(config);
  }

  /**
   * Configures the hood motor
   */
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID and feedforward gains
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = HoodConstants.kS;
    slot0.kV = HoodConstants.kV;
    slot0.kA = HoodConstants.kA;
    slot0.kP = HoodConstants.kP;
    slot0.kI = HoodConstants.kI;
    slot0.kD = HoodConstants.kD;

    // Motor Inversion
    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED 
      ? InvertedValue.Clockwise_Positive 
      : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.HOOD_CURRENT_LIMIT > 0;

    // --- GEARING CONFIGURATION ---
    
    config.Feedback.SensorToMechanismRatio = 2 * (HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO);
    config.Feedback.RotorToSensorRatio = (44.0/9.0); 

    // Soft limits (Converted to Rotations: Degrees / 360)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 37.0 / 360.0; 
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0 / 360.0; 

    hoodMotor.getConfigurator().apply(config.withSlot0(slot0));
  }

  /**
   * Set hood goal based on ShotCalculator output
   */
  public void setGoalFromCalculator(ShotCalculator calculator) {
    setGoal(calculator.getHoodAngle());
  }

  public void manualUp() {
    // 0.5 degrees per loop
    setGoal(getGoalValue() + 0.5);
  }

  public void manualDown() {
    // 0.5 degrees per loop
    setGoal(getGoalValue() - 0.5);
  }

  /**
   * Set the target angle for the hood using trapezoidal motion profiling
   * @param goalDeg Target angle in degrees
   */
  public void setGoal(double goalDeg) {
    double clampedGoal = MathUtil.clamp(
        goalDeg, 
        HoodConstants.HOOD_MIN_ANGLE, 
        HoodConstants.HOOD_MAX_ANGLE
    );
    targetAngle = clampedGoal;
    
    // Convert degrees to rotations (0.0 - 1.0)
    hoodGoalState = new TrapezoidProfile.State(clampedGoal / 360.0, 0);
  }

  public double getGoalValue() {
    return hoodGoalState.position * 360.0;
  }

  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    hoodSetpointState = nextSetpoint;
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  /**
   * Returns current hood angle from motor encoder (degrees)
   */
  public double getCurrentAngle() {
    // Phoenix 6 returns "Mechanism Rotations" because we set SensorToMechanismRatio.
    // So we just multiply by 360 to get degrees.
    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  /**
   * Returns current hood angle from absolute encoder (degrees)
   */
  public double getAbsoluteAngle() {
    // CANcoder returns rotations (0-1).
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public boolean atTarget(double tolerance) {
    return Math.abs(getCurrentAngle() - targetAngle) < tolerance;
  }

  /**
   * Reset encoder to a specific angle
   */
  public void resetEncoderToAngle(double currentAngleDeg) {
    // Convert degrees to mechanism rotations
    double mechanismRotations = currentAngleDeg / 360.0;
    hoodMotor.setPosition(mechanismRotations);
  }

  private void setControl() {
    TrapezoidProfile.State targetState = hoodTrapezoidProfile.calculate(
      0.02, 
      hoodSetpointState,
      hoodGoalState
    );

    setSetpoint(targetState);

    // Because we configured SensorToMechanismRatio in configureMotor(),
    // we pass 'targetState.position' directly (it is already in mechanism rotations).
    // DO NOT multiply by gear ratio here.
    positionControl.Position = targetState.position;
    positionControl.Velocity = targetState.velocity;
    hoodMotor.setControl(positionControl);
  }

  @Override
  public void periodic() {
    setControl();

    // Telemetry
    double currentAngle = getCurrentAngle();
    double absoluteAngle = getAbsoluteAngle();

    SmartDashboard.putNumber("Hood/Current Angle (Motor)", currentAngle);
    SmartDashboard.putNumber("Hood/Absolute Angle (Encoder)", absoluteAngle);
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putNumber("Hood/Goal Angle", getGoalValue());
    SmartDashboard.putBoolean("Hood/At Target", atTarget(1.0));

    // Update Visualization 
    hoodArm.setAngle(currentAngle);

    // Motor debugging info
    SmartDashboard.putNumber("Hood/Motor Voltage (V)", hoodMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Encoder Discrepancy", Math.abs(currentAngle - absoluteAngle));
  }

  @Override
  public void simulationPeriodic() {
    simPosition = hoodSetpointState.position; 
    simVelocity = hoodSetpointState.velocity; 
    
    // Sim state expects Rotor Rotations, so we must multiply back by Gear Ratio here
    hoodMotor.getSimState().setRawRotorPosition(simPosition * HoodConstants.HOOD_GEAR_RATIO);
    hoodMotor.getSimState().setRotorVelocity(simVelocity * HoodConstants.HOOD_GEAR_RATIO);
    
    // Update CANcoder sim
    absoluteEncoder.getSimState().setRawPosition(simPosition);
  }
}