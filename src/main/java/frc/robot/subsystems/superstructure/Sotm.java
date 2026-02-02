package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GroundIntakeConstants;

import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.GroundIntake;
import frc.robot.ManualControls;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Sotm extends SubsystemBase {
    private final SwerveSubsystem swerve;
    private final Flywheel flywheel;
    private final Turret turret;
    private final ShooterLUT shooterLUT;
    private final Hood hood;
    private Pose2d fieldTarget;

    public Sotm(
        SwerveSubsystem swerve,
        Flywheel flywheel,
        Turret turret,
        ShooterLUT shooterLUT,
        Hood hood
        ) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterLUT = shooterLUT;
        this.hood = hood;
    }

    public void ShootBall() {
        // Allow runtime override to ignore field-based limits (useful for testing).
        // Toggle from SmartDashboard: "Shooting/IgnoreFieldLimits" (default false).
        boolean ignoreLimits = false;
        try {
            ignoreLimits = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Shooting/IgnoreFieldLimits", false);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Sotm/IgnoreFieldLimits", ignoreLimits);
        } catch (Exception e) {
            // SmartDashboard may not be available in unit tests; ignore.
        }

        // If ignoreLimits is true, always attempt to shoot (bypass field checks).
        if (ignoreLimits) {
            // NOTE: This forces shooting behavior regardless of robot position/alliance.
            shootState();
            return;
        }

        // If you prefer to permanently bypass the limits in code, you can replace the
        // entire method body with the following (commented-out) lines for quick testing:
        // --------- START ALWAYS-SHOOT PLACEHOLDER ---------
        // // shootState();
        // // return;
        // --------- END ALWAYS-SHOOT PLACEHOLDER ---------

        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d robotPose = swerve.getPose();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            if (robotPose.getX() < 4.03) {
                shootState();
            } else if (robotPose.getX() > 4.03 && robotPose.getX() < 16.45-4.03) {
                passState();
            } else {
                idleState();
            }
        } else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (robotPose.getX() > 16.54-4.03) {
                shootState();
            } else if (robotPose.getX() < 16.54-4.03 && robotPose.getX() > 4.03) {
                passState();
            } else {
                idleState();
            }
        } else {
            idleState();
        }
    }

    public void initialize() {
        fieldTarget = getHubPose(DriverStation.getAlliance());
    }

    Pose2d getHubPose(Optional<Alliance> alliance) {
        
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.HUB_RED_POSITION;
        }else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FieldConstants.HUB_BLUE_POSITION;
        } else{
        return FieldConstants.HUB_BLUE_POSITION;
        }
    }

    Pose2d getPassPose(Optional<Alliance> alliance) {
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.PASS_RED_POSITION;
        }else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FieldConstants.PASS_BLUE_POSITION;
        } else{
        return FieldConstants.PASS_BLUE_POSITION;
        }
    }

    public void shootState() {
        Pose2d robotPose = swerve.getPose();
        Pose2d fieldTarget = getHubPose(DriverStation.getAlliance());
        BallOut(robotPose, fieldTarget);
    }

    public void passState() {
        Pose2d robotPose = swerve.getPose();
        Pose2d fieldTarget = getPassPose(DriverStation.getAlliance());
        BallOut(robotPose, fieldTarget);
    }

    public void idleState() {
        flywheel.setRPM(Constants.FlywheelConstants.FLYWHEEL_IDLE_RPM);
        // Alphabot has no turret/hood: keep flywheel idle and don't command non-existent mechanisms
    }



    public void BallOut(Pose2d robotPose, Pose2d fieldTarget) {
        ChassisSpeeds robotSpeeds = swerve.getRobotVelocity();

        double rx = robotSpeeds.vxMetersPerSecond; // forward
        double ry = robotSpeeds.vyMetersPerSecond; // left

        double dx = robotPose.getX() - fieldTarget.getX();
        double dy = robotPose.getY() - fieldTarget.getY();
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        double launchAngle = shooterLUT.getAngle(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        double launchRPM = shooterLUT.getRPM(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        double launchVelocity = shooterLUT.getVelocity(launchRPM);

        double vk = launchVelocity * Math.sin(Math.toRadians(launchAngle));
        double vij = launchVelocity * Math.cos(Math.toRadians(launchAngle));
        double vi = vij * Math.cos(Math.toRadians(robotHeadingDeg));
        double vj = vij * Math.sin(Math.toRadians(robotHeadingDeg));

        ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        double ri = fieldRelative.vxMetersPerSecond;
        double rj = fieldRelative.vyMetersPerSecond;

        double finalVi = vi-ri;
        double finalVj = vj-rj;
        double finalVk = vk;

        double finalVelocity = Math.sqrt(Math.pow(finalVi,2) + Math.pow(finalVj,2) + Math.pow(finalVk,2));
        double finalRPM = shooterLUT.getRPMForVelocity(finalVelocity);
        double turretShootAngle = Math.toDegrees(Math.atan2(finalVj, finalVi));

        //IF YOU OVERSHOOT THEN RECALCULATE THE ANGLE BASED ON THE INVERSE TANGENT OF THE VELOCITY COMPONENTS
        if (finalRPM > FlywheelConstants.FLYWHEEL_MAX_RPM) {
            flywheel.setRPM(FlywheelConstants.FLYWHEEL_MAX_RPM);
        } else {
            flywheel.setRPM(2500);

        }

    // Align drivetrain to computed turret shooting angle (use odometry-based pose estimator)
    swerve.alignRotationCommand(turretShootAngle);
    // Debug telemetry
    SmartDashboard.putNumber("Sotm/robotPoseX", robotPose.getX());
    SmartDashboard.putNumber("Sotm/robotPoseY", robotPose.getY());
    SmartDashboard.putNumber("Sotm/dx", dx);
    SmartDashboard.putNumber("Sotm/dy", dy);
    SmartDashboard.putNumber("Sotm/robotHeadingDeg", robotHeadingDeg);
    SmartDashboard.putNumber("Sotm/launchAngle", launchAngle);
    SmartDashboard.putNumber("Sotm/launchRPM", launchRPM);
    SmartDashboard.putNumber("Sotm/finalRPM", finalRPM);
    SmartDashboard.putNumber("Sotm/finalVelocity", finalVelocity);
    SmartDashboard.putNumber("Sotm/turretShootAngle", turretShootAngle);
    SmartDashboard.putNumber("Sotm/robotVx", robotSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Sotm/robotVy", robotSpeeds.vyMetersPerSecond);
    }

}