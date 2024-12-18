package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Worm;

public class WormSetPowerCommand extends CommandBase {
    private Worm worm;
    private final double power;

    public WormSetPowerCommand(Worm worm, double power) {
        this.worm = worm;
        this.power = power;
        addRequirements(worm);
    }

    @Override
    public void execute() {
        worm.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        worm.brake();
    }
}
