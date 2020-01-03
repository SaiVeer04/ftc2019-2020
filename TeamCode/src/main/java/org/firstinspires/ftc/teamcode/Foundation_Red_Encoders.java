package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Foundation")
public class Foundation_Red_Encoders extends LinearOpMode {
    componentsEncoder robot = new componentsEncoder();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        //go to foundation
        robot.forward(47.25,.5);

        //rotate servo down(nigga cat)


        //rotate to put foundation int building zone
        robot.turn(90,"r",.4);

        //nigga unlatch


        //move backwards
        robot.backwards(60,.5);


    }
}
