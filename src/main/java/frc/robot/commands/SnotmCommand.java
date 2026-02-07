package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.superstructure.Snotm;
import frc.robot.ManualControls;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.Flywheel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SnotmCommand extends Command {

    private final Snotm snotm;
    // kept references for future expansion (e.g., button-based overrides)
    private final ManualControls controls;
    private final Feeder feeder;
    private final Flywheel flywheel;

    public SnotmCommand(Snotm snotm, ManualControls controls, Feeder feeder, Flywheel flywheel) {
        this.snotm = snotm;
        this.controls = controls;
        this.feeder = feeder;
        this.flywheel = flywheel;
        addRequirements(snotm);
    }
    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Run shooting state machine every scheduler cycle so limelight alignment and
        // flywheel RPM calculation run continuously.
        snotm.ShootBall();
    }


}
