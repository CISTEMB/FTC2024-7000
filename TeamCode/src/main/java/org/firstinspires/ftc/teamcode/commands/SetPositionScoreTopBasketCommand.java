package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;
import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class SetPositionScoreTopBasketCommand extends CommandBase {
    private ElevatorV2 elevatorV2;
    private Worm worm;

    public SetPositionScoreTopBasketCommand(ElevatorV2 elevatorV2, Worm worm) {
        this.elevatorV2 = elevatorV2;
        this.worm = worm;

        addRequirements(elevatorV2, worm);
    }

    @Override
    public void execute() {
        SequentialCommandGroup command = new SequentialCommandGroup(
                new RunCommand(() -> worm.goToAngle(65)),
                new ElevatorV2ExtendCommand(elevatorV2, 1).interruptOn(() -> elevatorV2.isExtended())
        );
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        worm.brake();
        elevatorV2.setBrake();
    }
}
