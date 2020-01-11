package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

    public ColorSensor colorSensor;

    public ColorSensor bottomSensor;

    public CRServo leftServo;

    public CRServo rightServo;

    public DcMotor intakeLeft;
    //intakeright
    public DcMotor intakeRight;

    public DcMotor string;
    //rotate motor
    public DcMotor rotate;
    // Color sensor

    public DigitalChannel touchSensor;

    public DistanceSensor sensorRange;
    public DistanceSensor sensorRange1;


    HardwareMap hwMap = null;


    public void init(HardwareMap hwMap) {
        //init
        fr = hwMap.dcMotor.get("FR");
        fl = hwMap.dcMotor.get("FL");
        br = hwMap.dcMotor.get("BR");
        bl = hwMap.dcMotor.get("BL");
       // touch = hwMap.get(DigitalChannel.class,"sensor_digital");
        //sensorDistance = hwMap.get(DistanceSensor.class, "stone");

        //extra motors
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        bottomSensor = hwMap.get(ColorSensor.class, "bottomSensor");

        // get a reference to the distance sensor that shares the same name.

        leftServo = hwMap.get(CRServo.class, "leftServo");

        rightServo = hwMap.get(CRServo.class, "rightServo");

        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeRight = hwMap.dcMotor.get("intakeRight");

        string = hwMap.dcMotor.get("string");
        rotate = hwMap.dcMotor.get("rotate");

        touchSensor = hwMap.get(DigitalChannel.class,"touchSensor");

        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        sensorRange1 = hwMap.get(DistanceSensor.class, "sensor_range1");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor)sensorRange1;


        //


        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}