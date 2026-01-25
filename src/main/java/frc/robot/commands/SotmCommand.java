package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.superstructure.Sotm;
import frc.robot.ManualControls;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
// import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.ShooterLUT;
// import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Feeder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SotmCommand extends Command {

    private final Sotm sotm;
    private final Flywheel flywheel;
    private final SwerveSubsystem swerve;
    private final ShooterLUT lut;

    public SotmCommand(Sotm sotm, Flywheel flywheel, SwerveSubsystem swerve, ShooterLUT lut) {
        this.sotm = sotm;
        this.flywheel = flywheel;
        this.swerve = swerve;
        this.lut = lut;
        addRequirements(sotm);
    }
    

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    public void periodic() {
        Pose2d robotPose = swerve.getPose();
        Pose2d fieldTarget = FieldConstants.HUB_RED_POSITION;
        ChassisSpeeds robotSpeeds = swerve.getRobotVelocity();

        double rx = robotSpeeds.vxMetersPerSecond; // forward
        double ry = robotSpeeds.vyMetersPerSecond; // left

        double dx = robotPose.getX() - fieldTarget.getX();
        double dy = robotPose.getY() - fieldTarget.getY();
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        double finalRPM = lut.getRPM(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        if (finalRPM > FlywheelConstants.FLYWHEEL_MAX_RPM) {
            flywheel.setRPM(FlywheelConstants.FLYWHEEL_MAX_RPM);
        } else {
            flywheel.setRPM(finalRPM);
        }
        
    }

    
}
