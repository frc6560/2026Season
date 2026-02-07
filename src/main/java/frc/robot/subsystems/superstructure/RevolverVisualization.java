package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RevolverVisualization {

    private final RevolverSubsystem revolver;

    private final Mechanism2d mech2d;
    private final MechanismRoot2d root;

    private final MechanismLigament2d panArm;
    private final MechanismLigament2d pusherArm;

    private double panAngleDeg = 0.0;
    private double pusherAngleDeg = 0.0;

    private static final double PUSHER_OFFSET_DEG = 90.0;
    private static final double SCALE_FACTOR = 6.0; // RPM → deg/sec

    public RevolverVisualization(RevolverSubsystem revolver) {
        this.revolver = revolver;

        mech2d = new Mechanism2d(3, 3);
        root = mech2d.getRoot("RevolverRoot", 1.5, 1.5);

        panArm = root.append(new MechanismLigament2d("Pan", 1.0, 0));
        pusherArm = root.append(new MechanismLigament2d("Pusher", 0.6, PUSHER_OFFSET_DEG));

        SmartDashboard.putData("Revolver 2D", mech2d);
    }

    public void update() {
        double dt = 0.02; // 20 ms loop

        double panRPM = revolver.getPanActualRPM();
        double pusherRPM = revolver.getPusherActualRPM();

        panAngleDeg += panRPM * SCALE_FACTOR * dt;
        pusherAngleDeg += pusherRPM * SCALE_FACTOR * dt;

        panAngleDeg %= 360.0;
        pusherAngleDeg %= 360.0;

        panArm.setAngle(panAngleDeg);
        pusherArm.setAngle(pusherAngleDeg + PUSHER_OFFSET_DEG);
    }
}