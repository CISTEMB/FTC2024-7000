package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;

public class ElevatorV2RetractCommand extends CommandBase {
    private ElevatorV2 elevatorV2;

    public ElevatorV2RetractCommand(ElevatorV2 elevatorV2) {
        this.elevatorV2 = elevatorV2;
        addRequirements(elevatorV2);
    }

    @Override
    public void execute() {
        elevatorV2.setPower(-.5);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorV2.setBrake();
    }
}
