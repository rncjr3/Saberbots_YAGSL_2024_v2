// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.HangerSubsystem;
import frc.robot.subsystems.swervedrive.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  private final HangerSubsystem hang = new HangerSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private RevBlinking ledRevBlinking = new RevBlinking();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  
  private SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Score Amp");
  private Command m_autoSelected;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("shootAMP", new InstantCommand(() -> {
      m_ShooterSubsystem.shootAMP();
    }));

    NamedCommands.registerCommand("stopShooter", new InstantCommand(() -> {
      m_ShooterSubsystem.stopShooter();
    }));

    NamedCommands.registerCommand("shootSpeaker", new InstantCommand(() -> {
      m_ShooterSubsystem.shootSpeaker();
    }));

    SmartDashboard.putData("Auto choices", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
    OperatorConstants.LEFT_Y_DEADBAND),
    () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
    OperatorConstants.LEFT_X_DEADBAND),
    () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
    OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.getHID()::getYButtonPressed,
    driverXbox.getHID()::getAButtonPressed,
    driverXbox.getHID()::getXButtonPressed,
    driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), 0.4));
        //() -> driverXbox.getRightX() * 0.8);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    // Gyro keybind
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // Hanging keybinds
    driverXbox.leftBumper().onTrue(new InstantCommand(() -> hang.pullDown()));
    driverXbox.rightBumper().onTrue(new InstantCommand(() -> hang.pullUP()));
    driverXbox.y().onTrue(new InstantCommand(() -> hang.stopPull()));

    // Shooter keybinds
    driverXbox.povUp().onTrue(new InstantCommand(() -> m_ShooterSubsystem.shootSpeaker()));
    driverXbox.povRight().onTrue(new InstantCommand(() -> m_ShooterSubsystem.shootAMP()));
    driverXbox.povLeft().onTrue(new InstantCommand(() -> m_ShooterSubsystem.stopShooter()));
    driverXbox.povDown().onTrue(new InstantCommand(() -> m_ShooterSubsystem.intake()));

    // Light keybind
    driverXbox.x().onTrue(new InstantCommand(() -> {
      System.out.println("Lights set to blue");
      this.ledRevBlinking.setLightsToBlue();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_autoSelected = autoChooser.getSelected();

    return m_autoSelected;
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Sets the lights to blue.
   */
  public class RevBlinking {

    /**
     * Creates a Spark object for the rev blinking led strip.
     */
    private Spark ledLight = new Spark(0);

    /** 
     * Constructor for this RevBlinking.     * Sets the lights to default color (white)
     */
    public RevBlinking() {
      // Constructor for future implementation.
      this.setDefaultLights();
    }

    /**
     * Sets the led lights to default (white).
     */
    public void setDefaultLights() {
      this.ledLight.set(.93);
    }

    /**
     * Sets the led lights to blue.
     */
    public void setLightsToBlue() {
      this.ledLight.set(87);
    }
  }
}
