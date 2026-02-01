package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Foot {

    private DcMotor foot;

    private enum State {
        UP(FOOT_UP_POWER),
        OFF(FOOT_OFF_POWER),
        DOWN(FOOT_DOWN_POWER);
    private double output = 0;
    private State(double percent){
        this.output = percent;
    }
    }
    private State state = State.OFF;

    public Foot(HardwareMap hw) {
        foot = hw.get(DcMotor.class, "foot");
        foot.setDirection(DcMotor.Direction.REVERSE);
        foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setState(State s) {
        state = s;
        foot.setPower(s.output);
    }
    public void raise(){
        setState(State.UP);
    }
    public void lower(){
        setState(State.DOWN);
    }
    public void stop(){
        setState(State.OFF);
    }

    public double getPower() { return foot.getPower(); }
}
