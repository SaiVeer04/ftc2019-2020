package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Bin Laden")

public class dualRed extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();

    public DcMotor bl;
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor br;

    float hsvValues[] = {0F, 0F, 0F};

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    static final double wheel_diameter = 4.0;
    static final double gear_ratio = 1.0;
    //for encoders
    static final double ticks = 537.6;
    static final double ticks_per_inch = (ticks * gear_ratio) / (wheel_diameter * Math.PI);


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        bl = hardwareMap.dcMotor.get("BL");
        fl = hardwareMap.dcMotor.get("FL");
        fr = hardwareMap.dcMotor.get("FR");
        br = hardwareMap.dcMotor.get("BR");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //required to tell using encoder
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart(); //when the driver clicks play
        backwards(31, 0.5);


    }

    public void reset_motor() {
        //resets encoders
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //this one lets each movement run to the full extent with out overriding
    public void powerBusy(double power) {
        //lets program run fully
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
        while ((fl.isBusy() && fr.isBusy()) && (bl.isBusy() && br.isBusy())) {
        }
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }

    public void forward(double front, double power) {
        int final_front = (int) Math.round(front * ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(final_front);
        bl.setTargetPosition(final_front);
        fr.setTargetPosition(final_front);
        br.setTargetPosition(final_front);
        powerBusy(power);

    }

    public void backwards(double back, double power) {

        int final_back = (int) Math.round(back * ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(-final_back);
        bl.setTargetPosition(-final_back);
        fr.setTargetPosition(-final_back);
        br.setTargetPosition(-final_back);
        powerBusy(power);

    }

    public void strafeleft(double back, double power) {

        int final_back = (int) Math.round(back * ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(-final_back);
        bl.setTargetPosition(final_back);
        fr.setTargetPosition(final_back);
        br.setTargetPosition(-final_back);
        powerBusy(power);

    }

    public void straferight(double back, double power) {

        int final_back = (int) Math.round(back * ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(final_back);
        bl.setTargetPosition(-final_back);
        fr.setTargetPosition(-final_back);
        br.setTargetPosition(final_back);
        powerBusy(power);

    }

    public void turn(double degrees, String l_or_r, double power) {
        if (l_or_r.equals("l")) {
            int leftvalue = (int) ((degrees / 360) * 1120);
            reset_motor();
            fl.setTargetPosition(-leftvalue);
            bl.setTargetPosition(-leftvalue);
            fr.setTargetPosition(leftvalue);
            br.setTargetPosition(leftvalue);
            powerBusy(power);

        } else if (l_or_r.equals("r")) {
            int leftvalue = (int) ((degrees / 360) * 1120);
            reset_motor();
            fl.setTargetPosition(leftvalue);
            bl.setTargetPosition(leftvalue);
            fr.setTargetPosition(-leftvalue);
            br.setTargetPosition(-leftvalue);
            powerBusy(power);

        }

    }
}
