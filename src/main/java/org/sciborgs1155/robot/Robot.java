package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;
import static org.sciborgs1155.robot.Constants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Annotations.Log;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Ports.OI;
import org.sciborgs1155.robot.commands.Autos;
import org.sciborgs1155.robot.commands.Sysid;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.pneumatics.forklift.Forklift;
import org.sciborgs1155.robot.pneumatics.hanger.Hanger;
import org.sciborgs1155.robot.wristedintake.intake.Intake;
import org.sciborgs1155.robot.wristedintake.wrist.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // SUBSYSTEMS
  private final Drive drive = Drive.create();
  private final Elevator elevator = Elevator.create();
  private final Forklift forklift = Forklift.create();
  private final Hanger hanger = Hanger.create();
  private final Wrist wrist = Wrist.create();
  private final Intake intake = Intake.create();

  // COMMANDS
  Sysid routine = new Sysid(drive, elevator, intake, wrist);
  //AUTO
  @Log.NT private SendableChooser<Command> autos = Autos.configureAutos(drive, elevator, hanger, forklift, intake, wrist);

  @Log.NT private double speedMultiplier = Constants.FULL_SPEED;

  /** The robot contains subsystems, OI devices, and commands. */
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureSubsystemDefaults();
    configureBindings();
  }

  /** Configures basic behavior during different parts of the game. */
  private void configureGameBehavior() {
    // Configure logging with DataLogManager, Monologue, FailureManagement, and URCL
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    FaultLogger.setupLogging();
    addPeriodic(FaultLogger::update, 1);

    if (isReal()) {
      URCL.start();
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /** Creates an input stream for a joystick. */
  private InputStream createJoystickStream(InputStream input, double maxSpeed, double maxRate) {
    return input
        .deadband(Constants.DEADBAND, 1)
        .negate()
        .scale(maxSpeed)
        .scale(() -> speedMultiplier)
        .signedPow(2)
        .rateLimit(maxRate);
  }

  /**
   * Configures subsystem default commands. Default commands are scheduled when no other command is
   * running on a subsystem.
   */
  private void configureSubsystemDefaults() {
    drive.setDefaultCommand( //TODO change back from commented after running all of the sysIds and confirmation of values
        drive.drive(
            createJoystickStream(
                driver::getLeftX,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
            createJoystickStream(
                driver::getLeftY,
                DriveConstants.MAX_SPEED.in(MetersPerSecond),
                DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
            // createJoystickStream(
            //     driver::getRightY,
            //     DriveConstants.MAX_SPEED.in(MetersPerSecond),
            //     DriveConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)),
            createJoystickStream(
                driver::getRightX,
                DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond),
                DriveConstants.MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)))));
    elevator.setDefaultCommand(elevator.moveToHeight());
    forklift.setDefaultCommand(forklift.retract());
    hanger.setDefaultCommand(hanger.retract());
    intake.setDefaultCommand(intake.setDesiredSpeed(3));
    wrist.setDefaultCommand(wrist.setDesiredAngle(Radians.of(Units.degreesToRadians(42))));
  }

  /** Configures trigger -> command bindings */
  private void configureBindings() {
    autonomous().whileTrue(Commands.deferredProxy(autos::getSelected));

    FaultLogger.onFailing(f -> Commands.print(f.toString()));

    driver
        .leftBumper()
        .or(driver.rightBumper())
        .onTrue(Commands.runOnce(() -> speedMultiplier = Constants.FULL_SPEED))
        .onFalse(Commands.runOnce(() -> speedMultiplier = Constants.SLOW_SPEED));
    operator.x().toggleOnTrue(elevator.setGoal(Meters.of(3)));
    operator.y().toggleOnTrue(elevator.setGoal(Meters.of(5)));
    operator
        .a()
        .onTrue(Commands.runOnce(() -> elevator.stop = true))
        .onFalse(Commands.runOnce(() -> elevator.stop = false));
    operator.x().toggleOnTrue(intake.setDesiredSpeed(6));
    operator.y().toggleOnTrue(intake.setDesiredSpeed(4));
    // operator.a().onTrue(routine.run().alongWith(Commands.runOnce(() -> System.out.println("Running sysids... "))));
    // operator.a().onTrue(routine.intake().alongWith(Commands.runOnce(() -> System.out.println("running sysid on intake"))));
  }
}
