package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Intake {

    private DcMotor intake;

    private enum State {
        IN(INTAKE_IN_POWER),
        OFF(INTAKE_OFF_POWER),
        OUT(INTAKE_OUT_POWER);
        double output = 0;
    private State(double percent){
       this.output = percent;
    }
    }
    private State state = State.OFF;

    public Intake(HardwareMap hw) {
        intake = hw.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void intake(){
        setState(State.IN);
    }
    public void outtake(){
        setState(State.OUT);
    }
    public void stop(){
        setState(State.OFF);
    }
    private void setState(State s) {
        state = s;
        intake.setPower(s.output);
    }

    public double getPower() { return intake.getPower(); }
}
