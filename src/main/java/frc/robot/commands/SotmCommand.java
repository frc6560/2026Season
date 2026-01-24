package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.superstructure.Sotm;
import frc.robot.ManualControls;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
// import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.ShooterLUT;
// import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Feeder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SotmCommand extends Command {

    private final Sotm sotm;

    public SotmCommand(Sotm sotm) {
        this.sotm = sotm;
        addRequirements(sotm);
    }
    

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    public void periodic() {
        sotm.ShootBall();
    }

    
}
