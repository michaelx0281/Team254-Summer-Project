package org.sciborgs1155.robot.commands; //TODO - FIX ALL OF THE WEIRD PLACEMENT OF AUTO FILES!!

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.drive.DriveConstants.Rotation;
import org.sciborgs1155.robot.drive.DriveConstants.Translation;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.pneumatics.forklift.Forklift;
import org.sciborgs1155.robot.pneumatics.hanger.Hanger;
import org.sciborgs1155.robot.wristedintake.intake.Intake;
import org.sciborgs1155.robot.wristedintake.wrist.Wrist;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public final class Autos implements Sendable {

  private static Optional<Rotation2d> rotation = Optional.empty();

  private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
  private static Map<String, Command> commands;

  public static SendableChooser<Command> configureAutos(
      Drive drive, Elevator elevator, Hanger hanger, Forklift forklift, Intake intake, Wrist wrist) {
    AutoBuilder.configureHolonomic(
      drive::pose,
      drive::resetOdometry,
      drive::getRobotRelativeChassisSpeeds,
      s -> drive.driveRobotRelative(s),
      new HolonomicPathFollowerConfig(
        new PIDConstants(Translation.P, Translation.I, Translation.D),
        new PIDConstants(Rotation.P, Rotation.I, Rotation.D),
        DriveConstants.MAX_SPEED.in(MetersPerSecond),
        DriveConstants.RADIUS.in(Meters),
        new ReplanningConfig()),
      () -> alliance() == Alliance.Red,
      drive);

    PPHolonomicDriveController.setRotationTargetOverride(() -> rotation);

    configureMap(drive, elevator, hanger, forklift, intake, wrist);

    NamedCommands.registerCommands(commands); // this might be very silly but it was still fun :D

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser("New Auto");
    chooser.addOption("no auto", Commands.none());
    // chooser.addOption("what?", Commands.run());

    return chooser;
  }
  
  private static void configureMap(Drive drive, Elevator elevator, Hanger hanger, Forklift forklift, Intake intake, Wrist wrist){
        // commands.put("elevator-high", 
        //   elevator.setGoal(Meters.of(3))
        //     .andThen(elevator.moveToHeight()));
        // commands.put("start-intake",
        //   intake.setDesiredSpeed(3));
        commands = Map.of(
          "elevator-high", elevator.setGoal(Meters.of(3)).andThen(elevator.moveToHeight()),
          "start-intake", intake.setDesiredSpeed(3)
          );

      }

  // public Command get() {
  //   return chooser.getSelected().get();
  // }

  @Override
  public void initSendable(SendableBuilder builder) {
    chooser.initSendable(builder);
  }
}
