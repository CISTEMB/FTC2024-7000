package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class WristSetAngleCommand extends CommandBase {

    private Wrist wrist;
    private double targetAngle;

    public WristSetAngleCommand(Wrist wrist, double angle) {
        this.wrist = wrist;
        this.targetAngle = angle;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.Active = true;
    }

    @Override
    public void execute() {
        wrist.setAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
