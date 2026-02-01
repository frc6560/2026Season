package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{
    private final TalonFX feedMotor;
    private boolean isFeeding = false;
    
    public Feeder() {
        this.feedMotor = new TalonFX(FeederConstants.MOTOR_ID, "rio");
    }
    public void feed() {
        feedMotor.set(0.2);
        isFeeding = true;
    }
    public void stop() {
        feedMotor.set(0);
        isFeeding = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Feeder/Is Feeding", isFeeding);
    }
}
