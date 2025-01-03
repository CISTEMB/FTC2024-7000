package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class SetPositionDefaultCommand extends CommandBase {
    private ElevatorV2 elevatorV2;
    private Worm worm;

    public SetPositionDefaultCommand(ElevatorV2 elevatorV2, Worm worm) {
        this.elevatorV2 = elevatorV2;
        this.worm = worm;
        addRequirements(elevatorV2, worm);
    }

    @Override
    public void execute() {
        SequentialCommandGroup command = new SequentialCommandGroup(
                new ElevatorV2RetractCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.isRetracted()),
                new RunCommand(() -> worm.goToAngle(0))
        );
    }

    @Override
    public void end(boolean interrupted) {
        worm.brake();
        elevatorV2.setBrake();
    }
}
