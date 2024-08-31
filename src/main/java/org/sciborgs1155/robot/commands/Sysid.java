package org.sciborgs1155.robot.commands;

import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.wristedintake.intake.Intake;
import org.sciborgs1155.robot.wristedintake.wrist.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



public class Sysid {
    /*All of the subsystem containing feedback and feedforward closed-loop control mechanism [VERY CURSED] */
    private Drive drive;
    private Elevator elevator;
    private Intake intake;
    private Wrist wrist;

    /* Just a class created to store methods to run every single sysid command */
    public Sysid(Drive drive, Elevator elevator, Intake intake, Wrist wrist) {
        this.drive = drive;
        this.elevator = elevator;
        this.intake = intake;
        this.wrist = wrist;
    }

    private Command runDynamicForward(){
        return  drive.driveSysIdDynamic(Direction.kForward).andThen(
                    drive.turnSysIdDynamic(Direction.kForward)).alongWith(
                        elevator.elevatorSysidDynamic(Direction.kForward)).alongWith(
                            intake.intakeSysidDynamic(Direction.kForward)).alongWith(
                                wrist.wristSysidDynamic(Direction.kForward));
    }

    private Command runDynamicBackward(){
        return  drive.driveSysIdDynamic(Direction.kReverse).andThen(
                    drive.turnSysIdDynamic(Direction.kReverse)).alongWith(
                        elevator.elevatorSysidDynamic(Direction.kReverse)).alongWith(
                            intake.intakeSysidDynamic(Direction.kReverse)).alongWith(
                                wrist.wristSysidDynamic(Direction.kReverse));
    }

    private Command runQuasistaticForward() {
        return  drive.driveSysIdQuasistatic(Direction.kForward).andThen(
                    drive.turnSysIdQuasistatic(Direction.kForward)).alongWith(
                        elevator.elevatorSysidQuasistatic(Direction.kForward)).alongWith(
                            intake.intakeSysidQuasistatic(Direction.kForward)).alongWith(
                                wrist.wristSysidQuasistatic(Direction.kForward));
    }

    private Command runQuasistaticBackward() {
        return  drive.driveSysIdQuasistatic(Direction.kReverse).andThen(
                    drive.turnSysIdQuasistatic(Direction.kReverse)).alongWith(
                        elevator.elevatorSysidQuasistatic(Direction.kReverse)).alongWith(
                            intake.intakeSysidQuasistatic(Direction.kReverse)).alongWith(
                                wrist.wristSysidQuasistatic(Direction.kReverse));
    }

    public Command run() {
        return runDynamicForward().andThen(
            runDynamicBackward()).andThen(
                runQuasistaticForward()).andThen(
                    runQuasistaticBackward()).andThen(
                        Commands.run(() -> System.out.println("Ended all routines"))
                    );
    }

    public Command intake() {
        return  intake.intakeSysidDynamic(Direction.kForward).andThen(
                    intake.intakeSysidDynamic(Direction.kReverse)).andThen(
                        intake.intakeSysidQuasistatic(Direction.kForward)).andThen(
                            intake.intakeSysidQuasistatic(Direction.kReverse)).andThen(
                                Commands.run(() -> System.out.println("Done setting up intake!"))
                            );
    }
}
