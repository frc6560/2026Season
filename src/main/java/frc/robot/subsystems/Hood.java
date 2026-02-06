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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  // Hardware
  private final TalonFX hoodMotor;
  private final DutyCycleEncoder absoluteEncoder;

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

    // Initialize hardware
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new DutyCycleEncoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID);

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

    if (isSimulation) {
      System.out.println("Hood initialized in SIMULATION mode");
    }
  }

  /**
   * Configures the absolute encoder (DutyCycleEncoder)
   */
  private void configureAbsoluteEncoder() {
    // DutyCycleEncoder reports rotations as a fraction [0,1) for one physical rotation.
    // Treat one full encoder rotation as 1.0 distance unit so getAbsolutePosition() * 360 gives degrees.
    absoluteEncoder.setDistancePerRotation(360.0);
    //absoluteEncoder.reset(); // or other config calls that your encoder supports

    // If you later switch to a CTRE CANcoder, apply a CANcoderConfiguration via the CANcoder API.
  }

  /**
   * Configures the hood motor with inversion set once
   */
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID and feedforward gains for position control
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = HoodConstants.kS;
    slot0.kV = HoodConstants.kV;
    slot0.kA = HoodConstants.kA;
    slot0.kP = HoodConstants.kP;
    slot0.kI = HoodConstants.kI;
    slot0.kD = HoodConstants.kD;

    // Set motor inversion ONCE in configuration
    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED 
      ? InvertedValue.Clockwise_Positive 
      : InvertedValue.CounterClockwise_Positive;

    // Set neutral mode to brake
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.HOOD_CURRENT_LIMIT > 0;

    // Gear ratios
    config.Feedback.SensorToMechanismRatio = 2 * (HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO);
    config.Feedback.RotorToSensorRatio = (44.0/9.0); 

     //soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 37.0 / 360.0 * HoodConstants.HOOD_GEAR_RATIO; //37 degrees
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0 / 360.0; //0 degrees

    // Apply configuration
    hoodMotor.getConfigurator().apply(config.withSlot0(slot0));
  }

  /**
   * Calculates distance from robot to target hub
   */
  private double getDistanceToTarget() {
    // This method is no longer used - ShotCalculator handles this
    // Kept for backwards compatibility if needed
    return 0.0;
  }

  /**
   * Gets the hub position based on alliance color
   */
  private Translation2d getHubPose() {
    // This method is no longer used - ShotCalculator handles this
    // Kept for backwards compatibility if needed
    return new Translation2d();
  }


  /**
   * Set hood goal based on ShotCalculator output
   * @param calculator The ShotCalculator instance
   */
  public void setGoalFromCalculator(ShotCalculator calculator) {
    setGoal(calculator.getHoodAngle());
  }
   public void manualUp() {
    // 0.5 degrees per 20ms = 25 degrees per second
    double nextAngle = getGoalValue() + 0.5;
    setGoal(nextAngle);
  }

  /**
   * Manually decreases the hood angle setpoint.
   */
  public void manualDown() {
    // 0.5 degrees per 20ms = 25 degrees per second
    double nextAngle = getGoalValue() - 0.5;
    setGoal(nextAngle);
  }

  /**
   * Set the target angle for the hood using trapezoidal motion profiling
   * @param goalDeg Target angle in degrees
   */
  public void setGoal(double goalDeg) {
    // Normalize angle to 0-360
    goalDeg = ((goalDeg % 360) + 360) % 360;
    targetAngle = goalDeg;
    
    // Convert degrees to rotations for the profile
    hoodGoalState = new TrapezoidProfile.State(goalDeg / 360.0, 0);
  }

  /**
   * Get the current goal state
   */
  public TrapezoidProfile.State getGoal() {
    return hoodGoalState;
  }

  /**
   * Get the goal angle in degrees
   */
  public double getGoalValue() {
    return hoodGoalState.position * 360.0;
  }

  /**
   * Set the current setpoint state
   */
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    hoodSetpointState = nextSetpoint;
  }

  /**
   * Get the current setpoint state
   */
  public TrapezoidProfile.State getSetpoint() {
    return hoodSetpointState;
  }

  /**
   * Stop hood movement
   */
  public void stop() {
    hoodMotor.stopMotor();
  }

  /**
   * Returns current hood angle from motor encoder (degrees)
   */
  public double getCurrentAngle() {
    double motorRotations = hoodMotor.getPosition().getValueAsDouble();
    return motorRotations * 360.0 / HoodConstants.HOOD_GEAR_RATIO;
  }

  /**
   * Returns current hood angle from absolute encoder (degrees)
   * Useful for verification and debugging
   */
  public double getAbsoluteAngle() {
    // Get position from Through Bore Encoder (0-1 rotation)
    double rawPosition = absoluteEncoder.get();
    
    // Apply offset and convert to degrees
    double position = rawPosition - HoodConstants.HOOD_ABSOLUTE_ENCODER_OFFSET;
    
    // Normalize to 0-1 range
    position = position % 1.0;
    if (position < 0) position += 1.0;
    
    // Convert to degrees (0-360)
    double angleDeg = position * 360.0;
    
    // Account for gear ratio if encoder is geared
    angleDeg = angleDeg / HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO;
    
    return angleDeg;
  }


  /**
   * Returns target angle (degrees)
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Returns distance to target (for telemetry)
   * @deprecated Use ShotCalculator.getDistance() instead
   */
  @Deprecated
  public double getDistance() {
    return getDistanceToTarget();
  }

  /**
   * Check if hood is at target position
   */
  public boolean atTarget(double tolerance) {
    return Math.abs(getCurrentAngle() - targetAngle) < tolerance;
  }

  /**
   * Reset the internal encoder to zero at the current position
   */
  public void resetEncoder() {
    hoodMotor.setPosition(0);
  }

  /** 
   * Reset encoder and set it to a specific angle
   * @param currentAngleDeg The actual current angle of the hood in degrees
   */
  public void resetEncoderToAngle(double currentAngleDeg) {
    // Convert degrees to mechanism rotations (SensorToMechanismRatio handles motor conversion)
    double mechanismRotations = currentAngleDeg / 360.0;
    hoodMotor.setPosition(mechanismRotations);
  }

  /**
   * Execute the trapezoidal motion profile control
   */
  private void setControl() {
    // Calculate the next state using trapezoidal profile
    TrapezoidProfile.State targetState = hoodTrapezoidProfile.calculate(
      0.02, // 20ms period
      hoodSetpointState,
      hoodGoalState
    );

    // targetState.position is already in mechanism rotations (0-1 = 0-360°)
    // No need to multiply by gear ratio - SensorToMechanismRatio handles it
    double targetRotations = targetState.position;
    double targetVelocity = targetState.velocity;

    // Update setpoint
    setSetpoint(targetState);

    // Apply position control with velocity feedforward
    positionControl.Position = targetRotations;
    positionControl.Velocity = targetVelocity;
    hoodMotor.setControl(positionControl);
  }

  @Override
  public void periodic() {
    double currentAngle = getCurrentAngle();
    double absoluteAngle = getAbsoluteAngle();

    // Update simulation if in sim mode
    if (isSimulation) {
      simulationPeriodic();
    }

    // Execute motion profile control
    setControl();

    // Update SmartDashboard
    SmartDashboard.putNumber("Hood/Current Angle (Motor)", currentAngle);
    SmartDashboard.putNumber("Hood/Absolute Angle (Encoder)", absoluteAngle);
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putNumber("Hood/Goal Angle", getGoalValue());
    SmartDashboard.putNumber("Hood/Angle Error", targetAngle - currentAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTarget(1.0));
    SmartDashboard.putBoolean("Hood/Is Simulation", isSimulation);

    // Motor debugging info
    SmartDashboard.putNumber("Hood/Motor Position (rotations)", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Velocity (rps)", hoodMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Voltage (V)", hoodMotor.getMotorVoltage().getValueAsDouble());

    // Profile state telemetry
    SmartDashboard.putNumber("Hood/Setpoint Position", hoodSetpointState.position * 360.0);
    SmartDashboard.putNumber("Hood/Setpoint Velocity", hoodSetpointState.velocity * 360.0);

    // Check encoder discrepancy
    double encoderDiscrepancy = Math.abs(currentAngle - absoluteAngle);
    SmartDashboard.putNumber("Hood/Encoder Discrepancy", encoderDiscrepancy);
    SmartDashboard.putBoolean("Hood/Needs Rehome", encoderDiscrepancy > 5.0);
  }

  /**
   * Simple simulation - just tracks the setpoint position
   */
  @Override
  public void simulationPeriodic() {
    // With SensorToMechanismRatio, sim position is in mechanism rotations
    simPosition = hoodSetpointState.position; // Already in mechanism rotations
    simVelocity = hoodSetpointState.velocity; // Already in mechanism rotations/sec

    // Debug output
    if (Math.random() < 0.02) { // Print ~once per second
      System.out.println(String.format("SIM: Goal=%.1f° Current=%.1f° SetpointPos=%.3f",
        getGoalValue(), getCurrentAngle(), hoodSetpointState.position));
    }

    // Feed simulated values back to the motor (in mechanism rotations)
    hoodMotor.getSimState().setRawRotorPosition(simPosition);
    hoodMotor.getSimState().setRotorVelocity(simVelocity);

    // Update CANcoder (which has different gearing: 18:44 = 2.44:1)
    double absPosition = (simPosition * HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO) % 1.0;
    if (absPosition < 0) absPosition += 1.0;
   // absoluteEncoder.getSimState().setRawPosition(absPosition);
  }
}