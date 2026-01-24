package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.superstructure.*;
import frc.robot.commands.*;

import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.Auto;
import frc.robot.autonomous.AutoFactory;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

    // Drivebase and vision
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/falcon")
    );
    private final VisionSubsystem vision;

    // Subsystems
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final BallGrabber ballGrabber = new BallGrabber();
    private final RevolverSubsystem revolver = new RevolverSubsystem();
    private final RevolverVisualization revolverViz = new RevolverVisualization(revolver);
    private final SubsystemManager subsystemManager = new SubsystemManager(drivebase, elevator, arm, ballGrabber, controls);

    // Autonomous
    private final AutoFactory factory;
    private final SendableChooser<Auto> autoChooser;

    // Drive input
    private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1
    )
    .withControllerRotationAxis(() -> -driverXbox.getRightX())
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

    public RobotContainer() {

        // Default commands
        arm.setDefaultCommand(new ArmCommand(arm, controls));
        elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));
        ballGrabber.setDefaultCommand(new BallGrabberCommand(ballGrabber, controls));
        subsystemManager.setDefaultCommand(
            new SubsystemManagerCommand(drivebase, elevator, arm, ballGrabber, controls, subsystemManager)
        );

        // Autonomous chooser
        factory = new AutoFactory(null, drivebase);
        autoChooser = new SendableChooser<>();
        for (AutoRoutines auto : AutoRoutines.values()) {
            Auto autonomousRoutine = new Auto(auto, factory);
            if (auto == AutoRoutines.TEST) {
                autoChooser.setDefaultOption(autonomousRoutine.getName(), autonomousRoutine);
            } else {
                autoChooser.addOption(autonomousRoutine.getName(), autonomousRoutine);
            }
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Vision subsystem
        List<LimelightVision> limelights = new ArrayList<>();
        for (String name : frc.robot.Constants.LimelightConstants.LIMELIGHT_NAMES) {
            Pose3d cameraPose = frc.robot.Constants.LimelightConstants.getLimelightPose(name);
            limelights.add(new LimelightVision(drivebase, name, cameraPose));
        }
        vision = new VisionSubsystem(limelights);

        // Button bindings
        configureBindings();

        // In simulation, ensure both motors spin at idle automatically
        if (RobotBase.isSimulation()) {
            revolver.startPan();      // Will start at target RPM, but idle will always show in visualization
            revolver.startPusher();
        }
    }

    private void configureBindings() {
        // Drive default command
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Driver buttons
        driverXbox.a().onTrue(
            Commands.defer(() -> Commands.runOnce(() -> vision.hardReset("limelight"), vision), Set.of(vision))
        );
        driverXbox.b().onTrue(Commands.runOnce(() -> drivebase.trackAprilTag().schedule(), drivebase));
        driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.alignToTrenchCommand()), drivebase));
        driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroNoAprilTagsGyro));
        driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverXbox.rightBumper().onTrue(Commands.none());

        // Operator buttons (secondXbox)
        new Trigger(() -> secondXbox.getRightTriggerAxis() > 0.5)
            .whileTrue(new RunRevolverCommand(revolver));

        new Trigger(() -> secondXbox.getBButton())
            .whileTrue(revolver.run(() -> revolver.startPusher()))
            .whileFalse(revolver.runOnce(() -> revolver.stopPusher()));

        new Trigger(() -> secondXbox.getXButton())
            .whileTrue(revolver.run(() -> revolver.startPan()))
            .whileFalse(revolver.runOnce(() -> revolver.stopPan()));

        new Trigger(() -> secondXbox.getAButton())
            .onTrue(revolver.runOnce(() -> revolver.stopBoth()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getCommand();
    }

    /**
     * Call this every loop from Robot.robotPeriodic() to update Mechanism2d sticks
     */
    public void updateVisualization() {
        revolverViz.update();
    }
}
