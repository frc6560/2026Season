package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  // Visualization
  private final Mechanism2d hoodMech;
  private final MechanismLigament2d hoodArm;

  // Control
  private final PositionVoltage positionControl;
  private final TrapezoidProfile.Constraints hoodConstraints;
  private final TrapezoidProfile hoodTrapezoidProfile;
  
  // State for Motion Profile
  private TrapezoidProfile.State hoodSetpointState = new TrapezoidProfile.State();
  private TrapezoidProfile.State hoodGoalState = new TrapezoidProfile.State();
  private double targetAngle = 0.0;

  public Hood(PoseSupplier poseSupplier) {
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_ABSOLUTE_ENCODER_ID);

    positionControl = new PositionVoltage(0.0).withSlot(0);

    // Profile Constraints
    hoodConstraints = new TrapezoidProfile.Constraints(
      HoodConstants.kMaxV,
      HoodConstants.kMaxA
    );
    hoodTrapezoidProfile = new TrapezoidProfile(hoodConstraints);

    configureAbsoluteEncoder();
    configureMotor();

    // Visualization
    hoodMech = new Mechanism2d(200, 200);
    MechanismRoot2d hoodRoot = hoodMech.getRoot("Hood Root", 100, 100);
    hoodArm = hoodRoot.append(new MechanismLigament2d("Hood Arm", 50, 0));
    hoodArm.setColor(new Color8Bit(255, 165, 0));
    SmartDashboard.putData("Hood Mechanism", hoodMech);
  }

  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0; 
    absoluteEncoder.getConfigurator().apply(config);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.Slot0.kS = HoodConstants.kS;
    config.Slot0.kV = HoodConstants.kV;
    config.Slot0.kA = HoodConstants.kA;
    config.Slot0.kP = HoodConstants.kP;
    config.Slot0.kI = HoodConstants.kI;
    config.Slot0.kD = HoodConstants.kD;

    config.MotorOutput.Inverted = HoodConstants.HOOD_MOTOR_INVERTED 
      ? InvertedValue.Clockwise_Positive 
      : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // --- GEARING CONFIGURATION (Matches your specific hardware) ---
    config.Feedback.SensorToMechanismRatio = 2 * (HoodConstants.ABSOLUTE_HOOD_ENCODER_GEAR_RATIO);
    config.Feedback.RotorToSensorRatio = (44.0/9.0); 

    // Soft limits (0 to 37 degrees)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 37.0 / 360.0; 
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0 / 360.0; 

    hoodMotor.getConfigurator().apply(config);
  }

  public void setGoalFromCalculator(ShotCalculator calculator) {
    setGoal(calculator.getHoodAngle());
  }

  public void manualUp() {
    setGoal(targetAngle + 0.5); // Nudge up
  }

  public void manualDown() {
    setGoal(targetAngle - 0.5); // Nudge down
  }

  public void setGoal(double goalDeg) {
    targetAngle = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    hoodGoalState = new TrapezoidProfile.State(targetAngle / 360.0, 0);
  }

  /**
   * SAFETY FIX: Syncs software to reality.
   * Prevents the hood from snapping if it fell while disabled.
   */
  public void resetProfileToCurrent() {
    double currentRotations = getCurrentAngle() / 360.0;
    hoodSetpointState = new TrapezoidProfile.State(currentRotations, 0);
    // Also reset the target to where we are now, so it holds position
    targetAngle = getCurrentAngle();
    hoodGoalState = new TrapezoidProfile.State(currentRotations, 0);
  }

  /**
   * SAFETY FIX: Calculates the profile and moves the motor.
   * Only called when the command is running.
   */
  public void runControlLoop() {
    // 1. Calculate next step
    hoodSetpointState = hoodTrapezoidProfile.calculate(0.02, hoodSetpointState, hoodGoalState);

    // 2. Send to motor
    positionControl.Position = hoodSetpointState.position;
    positionControl.Velocity = hoodSetpointState.velocity;
    hoodMotor.setControl(positionControl);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  public double getCurrentAngle() {
    return hoodMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public double getAbsoluteAngle() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  public boolean atTarget() {
    return Math.abs(getCurrentAngle() - targetAngle) < 1.0; 
  }

  @Override
  public void periodic() {
    // Only update Telemetry here. No movement.
    SmartDashboard.putNumber("Hood/Current Angle", getCurrentAngle());
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    
    hoodArm.setAngle(getCurrentAngle());
  }

  public interface PoseSupplier {
    Pose2d getPose();
  }
}