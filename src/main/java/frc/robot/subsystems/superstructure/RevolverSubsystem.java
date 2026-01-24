package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RevolverSubsystem extends SubsystemBase {

    /* ======================= Hardware ======================= */
    private final TalonFX panMotor = new TalonFX(10);
    private final TalonFX pusherMotor = new TalonFX(11);

    // Beam breaker (DIO)
    private final DigitalInput beamBreaker = new DigitalInput(0);

    /* ======================= RPM Targets ======================= */
    private static final double PAN_RUNNING_RPM = 180.0;
    private static final double PUSHER_RUNNING_RPM = 2500.0;
    private static final double IDLE_RPM = 10.0;

    private static final double PAN_GEAR_RATIO = 1.0;
    private static final double PUSHER_GEAR_RATIO = 1.0;

    /* ======================= Commanded State ======================= */
    private double panCommandedRPM = IDLE_RPM;
    private double pusherCommandedRPM = IDLE_RPM;

    /* ======================= Control ======================= */
    private final VelocityVoltage panRequest = new VelocityVoltage(0);
    private final VelocityVoltage pusherRequest = new VelocityVoltage(0);

    /* ======================= Constructor ======================= */
    public RevolverSubsystem() {
        configureMotor(panMotor, 0.25, 40);
        configureMotor(pusherMotor, 0.15, 60);
    }

    /* ======================= Motor Config ======================= */
    private void configureMotor(TalonFX motor, double kP, int currentLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = kP;
        config.Slot0.kV = 0.12;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;

        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /* ======================= Helpers ======================= */
    private double rpmToRps(double rpm, double gearRatio) {
        return (rpm / 60.0) / gearRatio;
    }

    /* ======================= Beam Breaker ======================= */
    public boolean hasBall() {
        // If inverted, change to: return !beamBreaker.get();
        return beamBreaker.get();
    }

    /* ======================= Control Logic ======================= */

    public void startPan() {
        panCommandedRPM = PAN_RUNNING_RPM;
        panMotor.setControl(
            panRequest.withVelocity(rpmToRps(panCommandedRPM, PAN_GEAR_RATIO))
        );
    }

    public void stopPan() {
        panCommandedRPM = IDLE_RPM;
        panMotor.setControl(
            panRequest.withVelocity(rpmToRps(panCommandedRPM, PAN_GEAR_RATIO))
        );
    }

    public void startPusher() {
        // ❗ Stop feeding if ball already staged
        if (hasBall()) {
            stopPusher();
            return;
        }

        pusherCommandedRPM = PUSHER_RUNNING_RPM;
        pusherMotor.setControl(
            pusherRequest.withVelocity(rpmToRps(pusherCommandedRPM, PUSHER_GEAR_RATIO))
        );
    }

    public void stopPusher() {
        pusherCommandedRPM = IDLE_RPM;
        pusherMotor.setControl(
            pusherRequest.withVelocity(rpmToRps(pusherCommandedRPM, PUSHER_GEAR_RATIO))
        );
    }

    public void startBoth() {
        startPan();
        startPusher();
    }

    public void stopBoth() {
        stopPan();
        stopPusher();
    }

    /* ======================= Feedback (FOR VIZ) ======================= */
    public double getPanRPM() {
        return panCommandedRPM;
    }

    public double getPusherRPM() {
        return pusherCommandedRPM;
    }

    public boolean panAtSpeed() {
        return Math.abs(panCommandedRPM - PAN_RUNNING_RPM) < 5.0;
    }

    public boolean pusherAtSpeed() {
        return Math.abs(pusherCommandedRPM - PUSHER_RUNNING_RPM) < 50.0;
    }

    /* ======================= Periodic ======================= */
    @Override
    public void periodic() {

        // Auto-stop pusher if ball reaches beam breaker
        if (hasBall() && pusherCommandedRPM > IDLE_RPM) {
            stopPusher();
        }

        SmartDashboard.putNumber("Revolver/Pan RPM", panCommandedRPM);
        SmartDashboard.putNumber("Revolver/Pusher RPM", pusherCommandedRPM);
        SmartDashboard.putBoolean("Revolver/Has Ball", hasBall());
    }
}
