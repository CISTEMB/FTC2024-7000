package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends SubsystemBase {

    public ServoEx servo;
    private Telemetry tm;
    public boolean Active = false;
    private double CurrentAngle;
    private int MinimumAngle = -150;
    private int MaximumAngle = 150;
    private double Speed = .25; //default speed is 1


    public Wrist(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto){
        tm = telemetry;
        servo = new SimpleServo(hardwareMap, "wrist", -150, 150);
        telemetry.addData("wrist servo instantiated", isAuto);
        telemetry.update();
        servo.setInverted(true);
        if (!isAuto) {

            servo.setPosition(0);
            CurrentAngle = 150;
        }
        servo.setInverted(false);
        //setAngle(0);
    }

    public void scoreBasket() {
        setAngle(-10);
    }

    public void addFifteen() {
        setAngle(0);
    }
    public void subFifteen() {
        setAngle(40);
    }

    public void SetSpeed(double speed) {
        Speed = speed;
    }
    public void Goto(int angle) {
        CurrentAngle = angle;
        setAngle(angle);
    }
    public void AddDegree() {
        if (Active && CurrentAngle < MaximumAngle) {
            CurrentAngle += Speed;
            setAngle(CurrentAngle);
        }
    }

    public void RemoveDegree() {
        if (Active && CurrentAngle > MinimumAngle) {
            CurrentAngle -= Speed;
            setAngle(CurrentAngle);
        }
    }

    public void setAngle(double angle){
        tm.addData("wrist set angle", angle);
        if (this.Active == true) {
            servo.turnToAngle(angle);
        }
    }
    public void open(){
        setAngle(200);
    }
    public void close(){
        setAngle(-25);
    }

    public double getAngle() {
        return servo.getAngle();
    }
    @Override
    public void periodic(){
        tm.addData("wrist active", this.Active);
        tm.addData("wrist angle", servo.getAngle());
    }
}