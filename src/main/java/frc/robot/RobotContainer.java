package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AmptrapConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;

import frc.robot.commands.PivotPID;
import frc.robot.commands.ProcessNote;
import frc.robot.commands.AmptrapPID;
import frc.robot.commands.AutoPivotPID;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.AmpTrap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

/*
 * FORWARD = DIRECTION OF NOTE SHOT
 * FORWARD = DIRECTION OF NOTE SHOT
 * FORWARD = DIRECTION OF NOTE SHOT
 * FORWARD = DIRECTION OF NOTE SHOT
 * FORWARD = DIRECTION OF NOTE SHOT
 * FORWARD = DIRECTION OF NOTE SHOT
 * FORWARD = DIRECTION OF NOTE SHOT
 */

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final Intake m_intaker = new Intake();

  private final Climb m_climber = new Climb();

  private final Pivot m_pivot = new Pivot();

  private final Shooter m_shooter = new Shooter();

  private final Indexer m_indexer = new Indexer();

  private final AmpTrap m_amptrap = new AmpTrap();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_subController = new CommandXboxController(OIConstants.kSubControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0);
    //CameraServer.startAutomaticCapture(1);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                (-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)) * DriveConstants.driveLimiter,
                (-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)) * DriveConstants.driveLimiter,
                (-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)) * DriveConstants.driveLimiter,
                true, true),
            m_robotDrive));

        //new RunCommand(() -> m_pivot.setMotor(0.3 * m_subController.getRightY()));
        
    m_chooser.setDefaultOption("Autoline Auto", this.auto1());
    m_chooser.addOption("Speaker Auto", this.speakerAuto());

    SmartDashboard.putData(m_chooser);
    SignalLogger.stop();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    
     //pivot
    //m_driverController.povUp().onTrue(new RunCommand(() -> m_pivot.raisePivot(), m_pivot));
    //m_driverController.povUp().onFalse(new RunCommand(() -> m_pivot.stopPivot(), m_pivot));
    //m_driverController.povDown().onTrue(new RunCommand(() -> m_pivot.lowerPivot(), m_pivot));
    //m_driverController.povDown().onFalse(new RunCommand(() -> m_pivot.stopPivot(), m_pivot));
    m_subController.rightStick().onTrue(new PivotPID(m_pivot, PivotConstants.speakerSetpoint));
    m_subController.povLeft().onTrue(new PivotPID(m_pivot, 0));

    m_driverController.y().onTrue(new AmptrapPID(m_amptrap, AmptrapConstants.sourcePos));
    m_driverController.a().onTrue(new AmptrapPID(m_amptrap, AmptrapConstants.loweredPos));
    
    m_subController.rightBumper().onTrue(new RunCommand(() -> m_intaker.spinOut(), m_intaker));
    m_subController.rightBumper().onFalse(new RunCommand(() -> m_intaker.stopIntake(), m_intaker));
    m_subController.leftBumper().onTrue(new RunCommand(() -> m_intaker.spinIn(), m_intaker));
    m_subController.leftBumper().onFalse(new RunCommand(() -> m_intaker.stopIntake(), m_intaker));

    m_subController.povUp().onTrue(new RunCommand(() -> m_climber.raiseClimb(), m_climber));
    m_subController.povDown().onTrue(new RunCommand(() -> m_climber.lowerClimb(), m_climber));
    m_subController.povUp().onFalse(new RunCommand(() -> m_climber.stopClimb(), m_climber));
    m_subController.povDown().onFalse(new RunCommand(() -> m_climber.stopClimb(), m_climber));

    m_subController.x().onTrue(new RunCommand(() -> m_amptrap.intakeRoller(), m_amptrap));
    m_subController.x().onFalse(new RunCommand(() -> m_amptrap.stopRoller(), m_amptrap));
    m_subController.b().onTrue(new RunCommand(() -> m_amptrap.outtakeRoller(), m_amptrap));
    m_subController.b().onFalse(new RunCommand(() -> m_amptrap.stopRoller(), m_amptrap));
    m_subController.a().onTrue(new RunCommand(() -> m_amptrap.lowerElevator(), m_amptrap));
    m_subController.a().onFalse(new RunCommand(() -> m_amptrap.stopElevator(), m_amptrap));
    m_subController.y().onTrue(new RunCommand(() -> m_amptrap.raiseElevator(), m_amptrap));
    m_subController.y().onFalse(new RunCommand(() -> m_amptrap.stopElevator(), m_amptrap));

    m_subController.rightTrigger(0.5).onTrue(new RunCommand(() -> m_amptrap.score(), m_amptrap));
    m_subController.rightTrigger(0.5).onTrue(new RunCommand(() -> m_amptrap.stopAll(), m_amptrap));

    m_driverController.povLeft().onTrue(new RunCommand(() -> m_amptrap.unlock(), m_amptrap));
    m_driverController.povRight().onTrue(new RunCommand(() -> m_amptrap.lock(), m_amptrap));

    //shooter
    m_subController.leftBumper().onTrue(new RunCommand(() -> m_shooter.goBackwards(), m_shooter));
    m_subController.leftBumper().onFalse(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));
    m_subController.leftStick().onTrue(new RunCommand(() -> m_shooter.speakerScoring(), m_shooter));
    m_subController.leftStick().onFalse(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));
    //m_subController.rightStick().onTrue(new RunCommand(() -> m_shooter.goBackwards(), m_shooter));
    //m_subController.rightStick().onFalse(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));

    m_subController.leftBumper().onTrue(new RunCommand(() -> m_indexer.reverseIndexer(), m_indexer));
    m_subController.leftBumper().onFalse(new RunCommand(() -> m_indexer.stopIndex(), m_indexer));
    m_subController.rightBumper().onTrue(new RunCommand(() -> m_indexer.spinIndexer(), m_indexer));
    m_subController.leftBumper().onFalse(new RunCommand(() -> m_indexer.stopIndex(), m_indexer));
    m_subController.rightBumper().onTrue(new RunCommand(() -> m_shooter.outTakeShooter(), m_shooter));
    m_subController.rightBumper().onFalse(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // Create config for trajectory
    //return null;
    //return auto1();
    return m_chooser.getSelected();
  }

  public final Command speakerAuto(){
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.7, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.4, 0, new Rotation2d(0)),
        config);

    Trajectory secondTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.4, 0, new Rotation2d(0)),
      List.of(new Translation2d(0.7, 0)),
      new Pose2d(0, 0, new Rotation2d(0)),
      config
    );

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        secondTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.

    return new SequentialCommandGroup( 
    new WaitCommand(2),
    new AutoPivotPID(m_pivot, PivotConstants.speakerSetpoint),
    new InstantCommand(() -> m_shooter.speakerScoring(), m_shooter),
    new WaitCommand(0.5),
    new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
    new AutoPivotPID(m_pivot, 0),
    new InstantCommand(() -> m_intaker.spinIn(), m_intaker),
    new InstantCommand(() -> m_shooter.goBackwards(), m_shooter),
    swerveControllerCommand, //move
    new WaitCommand(1),
    new InstantCommand(() -> m_intaker.stopIntake(), m_intaker), 
    new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
    swerveControllerCommand2, //move and intake second note
    new WaitCommand(0.75),
    new AutoPivotPID(m_pivot, PivotConstants.speakerSetpoint),
    new InstantCommand(() -> m_shooter.speakerScoring(), m_shooter),
    new WaitCommand(0.5),
    new AutoPivotPID(m_pivot, 0),
    new InstantCommand(() -> m_shooter.stopShooter(), m_shooter),
    new InstantCommand(() -> m_intaker.stopIntake(), m_intaker),
    new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }

  public final Command auto1(){
    System.out.println("running auto");
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.55, 0)),     
        new Pose2d(1.1, 0, new Rotation2d(0)),
        config);

    Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.5, 0)),  
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    //exampleTrajectory.concatenate(exampleTrajectory2);
      
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)));
    
  }
}