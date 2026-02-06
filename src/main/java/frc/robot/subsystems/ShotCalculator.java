// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;

/**
 * Subsystem responsible for calculating shot parameters (hood angle, flywheel RPM)
 * based on distance to target. Keeps all lookup tables and calculations in one place.
 */
public class ShotCalculator extends SubsystemBase {

  // Lookup tables
  private final InterpolatingDoubleTreeMap distanceToHoodAngle;
  private final InterpolatingDoubleTreeMap distanceToFlywheelRPM;

  // Pose supplier
  private final PoseSupplier poseSupplier;

  // Cached calculations
  private double currentDistance = 0.0;
  private double calculatedHoodAngle = 0.0;
  private double calculatedFlywheelRPM = 0.0;

  public interface PoseSupplier {
    Pose2d getPose();
  }

  /** Creates a new ShotCalculator. */
  public ShotCalculator(PoseSupplier poseSupplier) {
    this.poseSupplier = poseSupplier;

    // Initialize hood angle lookup table
    distanceToHoodAngle = new InterpolatingDoubleTreeMap();
    for (int i = 0; i < HoodConstants.DISTANCE_ANGLE_TABLE.length; i++) {
      distanceToHoodAngle.put(
        HoodConstants.DISTANCE_ANGLE_TABLE[i][0], 
        HoodConstants.DISTANCE_ANGLE_TABLE[i][1]
      );
    }

    // Initialize flywheel RPM lookup table
    distanceToFlywheelRPM = new InterpolatingDoubleTreeMap();
    for (int i = 0; i < FlywheelConstants.DISTANCE_RPM_TABLE.length; i++) {
      distanceToFlywheelRPM.put(
        FlywheelConstants.DISTANCE_RPM_TABLE[i][0],
        FlywheelConstants.DISTANCE_RPM_TABLE[i][1]
      );
    }
  }

  /**
   * Calculate distance to the target hub based on current pose
   */
  private double calculateDistanceToTarget() {
    Pose2d robotPose = poseSupplier.getPose();
    Translation2d hubPose = getHubPose();
    return robotPose.getTranslation().getDistance(hubPose);
  }

  /**
   * Get the hub position based on alliance color
   */
  private Translation2d getHubPose() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return FlywheelConstants.HUB_RED_POSITION;
    }
    return FlywheelConstants.HUB_BLUE_POSITION;
  }

  /**
   * Get the required hood angle for the current distance
   */
  public double getHoodAngle() {
    return calculatedHoodAngle;
  }

  /**
   * Get the required flywheel RPM for the current distance
   */
  public double getFlywheelRPM() {
    return calculatedFlywheelRPM;
  }

  /**
   * Get the current distance to target
   */
  public double getDistance() {
    return currentDistance;
  }

  /**
   * Check if we have a valid shot (within range of lookup table)
   */
  public boolean hasValidShot() {
    // Check if distance is within the bounds of our lookup tables
    return currentDistance >= 0.0 && currentDistance <= 5.0; // Adjust max range as needed
  }

  @Override
  public void periodic() {
    // Update distance and shot parameters every cycle
    currentDistance = calculateDistanceToTarget();
    calculatedHoodAngle = distanceToHoodAngle.get(currentDistance);
    calculatedFlywheelRPM = distanceToFlywheelRPM.get(currentDistance);

    // Telemetry
    SmartDashboard.putNumber("ShotCalc/Distance", currentDistance);
    SmartDashboard.putNumber("ShotCalc/Hood Angle", calculatedHoodAngle);
    SmartDashboard.putNumber("ShotCalc/Flywheel RPM", calculatedFlywheelRPM);
    SmartDashboard.putBoolean("ShotCalc/Valid Shot", hasValidShot());
  }
}