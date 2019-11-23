package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "SkystoneBlue")
public class SkystoneBlue extends LinearOpMode {
    componentsHardCode robot = new componentsHardCode();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        waitForStart();
        strafeLeft(0.8, 15500, 150); //strafe left to the stones
        moveForward(0.5, 1000, 500); //move forward to straighten the robot out
        moveBackWard(0.5, 100, 500); //move backward from wall


        while(true){
            sleep(2000);
            if(robot.stoneSensor.red() < 1 && robot.stoneSensor.blue() < 1 && robot.stoneSensor.green() < 1){
                stopRobot();
                strafeLeft(0.8, 200 , 100);
                robot.drag.setPower(-0.5);
                sleep(1000);
                break;
            }
            else{
                moveBackWard(.4,75,75);
            }
        }
        strafeRight(.8, 1850, 200); //strafe back to wall
        int count = 0;
        while (robot.stoneSensor.blue()<30){
            moveBackWard(.5, 150, 100);
            stopRobot();
            sleep(1000);
            if(robot.stoneSensor.blue()>=30){
                stopRobot();
                break;
            }


            count+=150;
        }
        moveBackWard(.5,750,3000);
        robot.drag.setPower(.5);
        sleep(1000);
        moveForward(.5,750+count-450,200);



    }
    public void moveForward(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void moveBackWard(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeLeft (double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(-power);
        robot.bl.setPower(power);
        robot.fr.setPower(power);
        robot.br.setPower(-power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void strafeRight(double power,int movement,int sleep) throws InterruptedException{
        robot.fl.setPower(power);
        robot.bl.setPower(-power);
        robot.fr.setPower(-power);
        robot.br.setPower(power);
        Thread.sleep(movement);
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
        sleep(sleep);
    }
    public void stopRoobot(){
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }

    public void stopRobot()
    {
        robot.fl.setPower(0);
        robot.bl.setPower(0);
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }
}
