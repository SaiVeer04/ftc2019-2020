
        package org.firstinspires.ftc.teamcode;

        import android.graphics.Color;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;

        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class componentsEncoder{

    // we use customary because we cool like that
    //all holding values
    static final double wheel_diameter = 4.0;
    static final double gear_ratio = 1.0;
    //for encoders
    static final double ticks = 537.6;
    static final double ticks_per_inch = (ticks * gear_ratio)/(wheel_diameter * Math.PI);

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

    public Servo leftServo;

    public Servo rightServo;

    public DcMotor intakeLeft;
    //intakeright
    public DcMotor intakeRight;

    public DcMotor string;
    //rotate motor
    public DcMotor rotate;
    // Color sensor

    public DigitalChannel touchSensor;





    HardwareMap hwMap =  null;


    public void init(HardwareMap hwMap){
        //init
        fr = hwMap.dcMotor.get("FR");
        fl = hwMap.dcMotor.get("FL");
        br = hwMap.dcMotor.get("BR");
        bl = hwMap.dcMotor.get("BL");



        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //required to tell using encoder
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //prevents motor from moving in init phase
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        bottomSensor = hwMap.get(ColorSensor.class, "bottomSensor");

        // get a reference to the distance sensor that shares the same name.

        leftServo = hwMap.get(Servo.class, "leftServo");

        rightServo = hwMap.get(Servo.class, "leftServo");

        intakeLeft = hwMap.dcMotor.get("intakeLeft");
        intakeRight = hwMap.dcMotor.get("intakeRight");

        string = hwMap.dcMotor.get("string");
        rotate = hwMap.dcMotor.get("rotate");

        touchSensor = hwMap.get(DigitalChannel.class,"touchSensor");

        touchSensor.setMode(DigitalChannel.Mode.INPUT);





    }
    //methods are pretty self explanatory
    public void reset_motor(){
        //resets encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //this one lets each movement run to the full extent with out overriding
    public void powerBusy(double power) {
        //lets program run fully
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        while ((fl.isBusy() && fr.isBusy())&&(bl.isBusy() && br.isBusy())){}
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void forward(double front,double power){
        int final_front = (int)Math.round(front*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(final_front);
        bl.setTargetPosition(-final_front);
        fr.setTargetPosition(final_front);
        br.setTargetPosition(-final_front);
        powerBusy(power);

    }
    public void backwards(double back,double power){

        int final_back = (int)Math.round(back*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(-final_back);
        bl.setTargetPosition(final_back);
        fr.setTargetPosition(-final_back);
        br.setTargetPosition(final_back);
        powerBusy(power);

    }
    public void strafeleft(double back,double power){

        int final_back = (int)Math.round(back*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(-final_back);
        bl.setTargetPosition(final_back);
        fr.setTargetPosition(final_back);
        br.setTargetPosition(-final_back);
        powerBusy(power);

    }
    public void straferight(double back,double power){

        int final_back = (int)Math.round(back*ticks_per_inch);

        reset_motor();
        fl.setTargetPosition(final_back);
        bl.setTargetPosition(-final_back);
        fr.setTargetPosition(-final_back);
        br.setTargetPosition(final_back);
        powerBusy(power);

    }

    public void turn(double degrees,String l_or_r,double power){
        if(l_or_r.equals("l")){
            int leftvalue = (int)((degrees/360)* 1120);

            fl.setTargetPosition(leftvalue);
            bl.setTargetPosition(leftvalue);
            fr.setTargetPosition(-leftvalue);
            br.setTargetPosition(-leftvalue);


        }else if(l_or_r.equals("r")){
            int leftvalue = (int)((degrees/360)* 1120);

            fl.setTargetPosition(-leftvalue);
            bl.setTargetPosition(-leftvalue);
            fr.setTargetPosition(leftvalue);
            br.setTargetPosition(leftvalue);


        }




    }

}

