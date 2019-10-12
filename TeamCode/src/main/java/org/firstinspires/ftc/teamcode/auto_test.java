package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class auto_test extends LinearOpMode {
        components robot = new components();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        //balsjacs;nxv;;
        waitForStart();
        robot.forward(10,.2);
    }
}
