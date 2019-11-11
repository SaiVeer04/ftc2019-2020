package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

public class componentsHardCode {


    //for motor variables
    //front right
    public DcMotor fr;
    //back right
    public DcMotor br;
    //front left
    public DcMotor fl;
    //back left
    public DcMotor bl;
    //intakeLeft
    public DcMotor intakeLeft;
    //intakeright
    public DcMotor intakeRight;
    //Servo paddle
    public Servo paddle;
    //Servo Drag
    public Servo drag;
    //spool motor
    public DcMotor string;
    //rotate motor
    public DcMotor rotate;
    // Color sensor
    public ColorSensor sensorColor;
    //
    public CRServo foundation;


    HardwareMap hwMap = null;


    public void init(HardwareMap hwMap) {
        //init
        fr = hwMap.dcMotor.get("FR");
        fl = hwMap.dcMotor.get("FL");
        br = hwMap.dcMotor.get("BR");
        bl = hwMap.dcMotor.get("BL");
        foundation = hwMap.crservo.get("foundation");

        //extra motors
        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeRight = hwMap.dcMotor.get("intakeRight");
        string = hwMap.dcMotor.get("string");
        rotate = hwMap.dcMotor.get("rotate");

        //Servo
        drag = hwMap.servo.get("drag");
        //ETC
        sensorColor = hwMap.colorSensor.get("sensorColor");


        //
        drag.setPosition(1);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}