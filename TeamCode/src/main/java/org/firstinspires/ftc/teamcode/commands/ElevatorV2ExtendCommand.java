package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorV2;

public class ElevatorV2ExtendCommand extends CommandBase {
    private ElevatorV2 elevatorV2;
    private double power = 1;

    public ElevatorV2ExtendCommand(ElevatorV2 elevatorV2, double power) {
        this.elevatorV2 = elevatorV2;
        this.power = power;
        addRequirements(elevatorV2);
    }

    @Override
    public void execute() {
        elevatorV2.setPower(this.power);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorV2.setBrake();
    }
}
