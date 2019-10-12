/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Base Auton", group="Iterative Opmode")
public class baseAuton extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static MediaPlayer mediaPlayer = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo StoneServoRight;  // close and open
    private Servo StoneServoLeft;   // close and open
    private DcMotor stoneLift;
    private DcMotor stoneTilt;
    private DcMotor autonStoneExt;
    private DcMotor autonStoneLift;
    private CRServo autonStoneServo;


    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    private static final double COUNTS_PER_MOTOR_REV_60 = 420;    // eg: Andymark Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        stoneLift = hardwareMap.get(DcMotor.class, "motor5");
        stoneTilt = hardwareMap.get(DcMotor.class, "motor6");
        autonStoneExt = hardwareMap.get(DcMotor.class, "motor7");
        autonStoneLift = hardwareMap.get(DcMotor.class, "motor8");
        StoneServoLeft = hardwareMap.servo.get("servo1");
        StoneServoRight = hardwareMap.servo.get("servo2");
        autonStoneServo = hardwareMap.crservo.get("servo6");

        //initialize components
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stoneTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stoneLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonStoneExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        autonStoneLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        StoneServoRight.setPosition(0.7);
        StoneServoLeft.setPosition(0.3);
        autonStoneServo.setPower(0);



        enableEncoders(); //enable the encoders
        runtime.reset();
        boolean runOnce = true;
        waitForStart();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        while (opModeIsActive() && runOnce) {
            //Instructions for the robot

            move(-28,0,0);
            move(0,-40, 0);
            //Make sure this code does not repeat
            runOnce = false;
        }
    }

    //forward/backward, side to side, turn
    private void move(float strafeY,float strafeX, float turn){
        int leftFrontNew;
        int leftBackNew;
        int rightFrontNew;
        int rightBackNew;

        if(strafeY!=0){
            //adding the distance to move in inches to current position
            leftFrontNew = leftFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if(strafeX!=0){
            leftFrontNew = leftFront.getCurrentPosition() + (int) (1.25*strafeX * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) (1.25*strafeX * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() - (int) (1.25*strafeX * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (1.25*strafeX * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if(turn!=0){
            leftFrontNew = leftFront.getCurrentPosition() - (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH) ;
            leftBackNew = leftBack.getCurrentPosition() - (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }



        while(leftFront.isBusy()) {
            telemetry.addData("LeftFontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("RightFontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
            telemetry.update();
            Thread.yield();
        }

        stopMotors();
        runWithEncoder();
    }
    private void movePos(int leftFrontNew, int leftBackNew, int rightFrontNew, int rightBackNew){
        leftFront.setTargetPosition(leftFrontNew);
        leftBack.setTargetPosition(leftBackNew);
        rightFront.setTargetPosition(rightFrontNew);
        rightBack.setTargetPosition(rightBackNew);

        runToPositionEncoder();
        leftFront.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        rightBack.setPower(DRIVE_SPEED);
    }




    private void enableEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        stoneTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        stoneLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        autonStoneExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        autonStoneLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        stoneTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        stoneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        autonStoneExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        autonStoneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void stopMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    public void runToPositionEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        stoneTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        stoneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        autonStoneExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        autonStoneLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void runWithEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        stoneTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        stoneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        autonStoneExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        autonStoneLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}