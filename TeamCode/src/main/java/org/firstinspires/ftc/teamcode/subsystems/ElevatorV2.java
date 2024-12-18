package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorV2 extends SubsystemBase {
    private Telemetry telemetry;
    private DcMotorEx motor;
    private TouchSensor bottomLimit;
    private double wormAngle;
    private double minimumDistance;

    private double targetDistance = 0;
    private boolean isHomed = false;

    private double currentPower = 0.5;


    public ElevatorV2(HardwareMap hm, Telemetry tm){
        wormAngle = 0; //the value of the worm angle so we can calculate max distance
        motor = hm.get(DcMotorEx.class, "Extend");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLimit = hm.get(TouchSensor.class, "Touch");
        telemetry = tm;
        minimumDistance = getDistance();
    }

    public void SetWormAngle(double angle) {
        wormAngle = angle;
    }

    public void setPower(double power) {
        if (power > 0 && !this.isExtended()) {
            telemetry.addData("attempting to set power", power);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(power);
        } else if (power < 0 && !this.isRetracted()) {
            telemetry.addData("attempting to set power", power);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(power);
        }
    }

    public int getDistance(){
        return motor.getCurrentPosition();
    }

    public double getDistanceInInches() {
        return getDistance() * 0.01229202524;
    }

    public boolean isRetracted() {
        return bottomLimit.isPressed();
    }

    public void setBrake() {

        if (this.isRetracted()) {
            //if we're touching, turn off power
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            //if we're not touching, give it a little power so it doesn't slip
            int curr = (int)Math.round(motor.getCurrentPosition());
            motor.setPower(0.01);
            motor.setTargetPosition(curr);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //this method returns true if we are maxed out on distance
    public boolean isExtended(){
        double horizontalExtension = getHorizontalExtension();
        return horizontalExtension >= 17;
    }

    public double getHorizontalExtension() {
        double elevatorDistance = getDistanceInInches();
        return (elevatorDistance * Math.abs(Math.cos(Math.toRadians(wormAngle))));
    }

    //this function fires every cycle, at about 50hz, so anything in here will effectively be the default state
    @Override
    public void periodic() {
        telemetry.addData("Elevator_Distance", getDistance());
        telemetry.addData("Elevator_Target_Distance", this.targetDistance);
        telemetry.addData("ElevatorDistanceInInches", getDistanceInInches());
        telemetry.addData("ElevatorHorizontal", getHorizontalExtension());
        telemetry.addData("Elevator_Is_Homed", isHomed);
//        telemetry.addData("Elevator_Bottom_Limit_Pressed", bottomLimit.isPressed());
        telemetry.addData("ElevatorIsRetracted", isRetracted());
        telemetry.addData("ElevatorIsExtended", isExtended());
        telemetry.addData("Elevator Power", motor.getPower());



//            //this will make the elevator retract
//            if (isHomed == false) {
//                targetDistance = 0;
//            }
//
//
//            //if the elevator is touching the bottom, and not attempting to move, reset the encoder
//            if (bottomLimit.isPressed() && Status != ElevatorStatus.Extending) {
//                isHomed = true;
//                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                targetDistance = minimumDistance = getDistance();
//            }
//
//        if (targetDistance > getDistance() && Status != ElevatorStatus.Extending) {
//            extend();
//        }
//
//        if (targetDistance < getDistance() && Status != ElevatorStatus.Retracting) {
//            retract();
//        }

    }
}