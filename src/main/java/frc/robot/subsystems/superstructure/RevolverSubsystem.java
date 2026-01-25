package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Revolver subsystem using a simple state machine:
 * IDLE -> SPINNING_UP -> FEEDING -> STAGED
 * Handles beam breaker and override button cleanly.
 */
public class RevolverSubsystem extends SubsystemBase {

    /* ======================= Hardware ======================= */
    private final TalonFX panMotor = new TalonFX(10);
    private final TalonFX pusherMotor = new TalonFX(11);
    private final DigitalInput beamBreaker = new DigitalInput(0); // DIO 0

    /* ======================= RPM Targets ======================= */
    private static final double PAN_RUNNING_RPM = 180.0;
    private static final double PUSHER_RUNNING_RPM = 2500.0;
    private static final double IDLE_RPM = 10.0;

    private static final double PAN_GEAR_RATIO = 1.0;
    private static final double PUSHER_GEAR_RATIO = 1.0;

    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);

    /* ======================= State Machine ======================= */
    public enum RevolverState {
        IDLE,        // Both motors at idle
        SPINNING_UP, // Pan spinning only
        FEEDING,     // Pan + pusher running
        STAGED,      // Pan spinning, ball held
        OVERRIDE     // Feed regardless of beam breaker
    }

    private RevolverState state = RevolverState.IDLE;

    /* ======================= Beam Breaker Override ======================= */
    private boolean beamBreakOverride = false;

    /* ======================= Motor Helpers ======================= */
    private double rpmToRps(double rpm, double gearRatio) {
        return (rpm / 60.0) / gearRatio;
    }

    private void setPanRPM(double rpm) {
        panMotor.setControl(panRequest.withVelocity(rpmToRps(rpm, PAN_GEAR_RATIO)));
    }

    private void setPusherRPM(double rpm) {
        pusherMotor.setControl(pusherRequest.withVelocity(rpmToRps(rpm, PUSHER_GEAR_RATIO)));
    }

    /* ======================= Constructor ======================= */
    public RevolverSubsystem() {
        configureMotor(panMotor, 0.25, 40);
        configureMotor(pusherMotor, 0.15, 60);
    }

    private void configureMotor(TalonFX motor, double kP, int currentLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kV = 0.12;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /* ======================= Public API ======================= */
    public boolean hasBall() {
        return beamBreaker.get(); // invert if needed
    }

    public void enableBeamBreakOverride() {
        beamBreakOverride = true;
        state = RevolverState.OVERRIDE;
    }

    public void disableBeamBreakOverride() {
        beamBreakOverride = false;
        if (hasBall()) {
            state = RevolverState.STAGED;
        } else if (state != RevolverState.IDLE) {
            state = RevolverState.FEEDING;
        }
    }

    public void requestFeed() {
        if (state == RevolverState.IDLE) {
            state = RevolverState.SPINNING_UP;
        }
    }

    public void requestStop() {
        state = RevolverState.IDLE;
    }

    public boolean panAtSpeed() {
        return true; // Placeholder: can replace with actual encoder check
    }

    /* ======================= State-based RPM Accessors ======================= */
    public double getPanRPM() {
        switch (state) {
            case IDLE:
                return IDLE_RPM;
            case SPINNING_UP:
            case FEEDING:
            case STAGED:
            case OVERRIDE:
                return PAN_RUNNING_RPM;
            default:
                return IDLE_RPM;
        }
    }

    public double getPusherRPM() {
        switch (state) {
            case IDLE:
            case SPINNING_UP:
            case STAGED:
                return IDLE_RPM;
            case FEEDING:
            case OVERRIDE:
                return PUSHER_RUNNING_RPM;
            default:
                return IDLE_RPM;
        }
    }

    /* ======================= Periodic State Machine ======================= */
    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                setPanRPM(IDLE_RPM);
                setPusherRPM(IDLE_RPM);
                break;

            case SPINNING_UP:
                setPanRPM(PAN_RUNNING_RPM);
                setPusherRPM(IDLE_RPM);
                if (panAtSpeed()) {
                    state = RevolverState.FEEDING;
                }
                break;

            case FEEDING:
                setPanRPM(PAN_RUNNING_RPM);
                if (!hasBall() || beamBreakOverride) {
                    setPusherRPM(PUSHER_RUNNING_RPM);
                } else {
                    setPusherRPM(IDLE_RPM);
                    state = RevolverState.STAGED;
                }
                break;

            case STAGED:
                setPanRPM(PAN_RUNNING_RPM);
                setPusherRPM(IDLE_RPM);
                break;

            case OVERRIDE:
                setPanRPM(PAN_RUNNING_RPM);
                setPusherRPM(PUSHER_RUNNING_RPM);
                break;
        }

        SmartDashboard.putString("Revolver/State", state.name());
        SmartDashboard.putBoolean("Revolver/Has Ball", hasBall());
        SmartDashboard.putBoolean("Revolver/Beam Override", beamBreakOverride);
        SmartDashboard.putNumber("Revolver/Pan RPM", getPanRPM());
        SmartDashboard.putNumber("Revolver/Pusher RPM", getPusherRPM());
    }

    /* ======================= Convenience for RunRevolverCommand ======================= */
    public void startBoth() {
        requestFeed();
    }

    public void stopBoth() {
        requestStop();
    }

    /* ======================= Accessors ======================= */
    public RevolverState getState() {
        return state;
    }
}
