package frc.robot.subsystems.superstructure;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLUT {

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap RPMtoVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap VelocitytoRPMMap = new InterpolatingDoubleTreeMap();

    public ShooterLUT() {

        //distance (meters) to RPM
        rpmMap.put(5.715, 2300.0);
        rpmMap.put(4.953, 1800.0);
        rpmMap.put(4.572, 1300.0);
        rpmMap.put(4.191, 1000.0);
        rpmMap.put(3.048, 800.0);

        // distance (meters) to angle (degrees)
        angleMap.put(0.0, 45.0);
        angleMap.put(50.0, 45.0);
        

        // RPM to velocity (m/s)
        RPMtoVelocityMap.put(2000.0, 7.572);
        //RPMtoVelocityMap.put(1500.0, 1.172); //probably wrong
        RPMtoVelocityMap.put(1000.0, 6.421);
        RPMtoVelocityMap.put(900.0, 6.163);
        RPMtoVelocityMap.put(500.0, 0.35);

        // Velocity to RPM (m/s)
        VelocitytoRPMMap.put(7.572, 2000.0);
        VelocitytoRPMMap.put(6.421, 1000.0);
        VelocitytoRPMMap.put(6.163, 900.0);
        VelocitytoRPMMap.put(0.35, 500.0);
        
    }


    public double getRPM(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    public double getAngle(double distanceMeters) {
        return angleMap.get(distanceMeters);
    }

    public double getVelocity(double distanceMeters) {
        return RPMtoVelocityMap.get(distanceMeters);
    }

    public double getRPMForVelocity(double velocity) {
        return VelocitytoRPMMap.get(velocity);
    }
}

