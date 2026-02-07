package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Catapult {

    public DcMotor c1, c2;

    public enum State {
        FIRING,
        RETURNING,
        HOLDING
    }

    public State state = State.FIRING;
    public ElapsedTime timer = new ElapsedTime();

    public Catapult(HardwareMap hw) {
        c1 = hw.get(DcMotor.class, "catapult1");
        c2 = hw.get(DcMotor.class, "catapult2");

        c1.setDirection(DcMotor.Direction.REVERSE);
        c2.setDirection(DcMotor.Direction.FORWARD);

        c1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        c2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    public void fire() {
        setState(State.FIRING);
    }
    public void load(){
        setState(State.RETURNING);
    }
    public void hold(){
        if(state!=State.RETURNING)
            setState(State.HOLDING);
    }

    public void update() {
        switch (state) {
            case FIRING:
                setUp();
                if (timer.seconds() >= FIRE_TIME) {
                    timer.reset();
                    state = State.RETURNING;
                }
                break;

            case RETURNING:
                setDown();
                if (timer.seconds() >= LOAD_TIME) {
                    timer.reset();
                    state = State.HOLDING;
                }
                break;

            case HOLDING:
                setHold();
                // Catapult ready for next fire
                break;
        }
    }

    private void setState(State state){
        if(this.state != state)
            timer.reset();
        this.state = state;

    }
    private void setUp() {
        c1.setPower(CATAPULT_UP_POWER);
        c2.setPower(CATAPULT_UP_POWER);
    }

    private void setDown() {
        c1.setPower(CATAPULT_DOWN_POWER);
        c2.setPower(CATAPULT_DOWN_POWER);
    }

    private void setHold() {
        c1.setPower(CATAPULT_HOLD_POWER);
        c2.setPower(CATAPULT_HOLD_POWER);
    }
}
