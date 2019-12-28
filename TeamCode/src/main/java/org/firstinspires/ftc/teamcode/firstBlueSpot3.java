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

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "First Blue Spot to Line", group = "Iterative Opmode")
public class firstBlueSpot3 extends LinearOpMode {
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    private static final double COUNTS_PER_MOTOR_REV_60 = 1680;    // eg: Andymark Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double WHEEL_DIAMETER_INCHES_Lift = 2.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double COUNTS_PER_INCH_60 = (COUNTS_PER_MOTOR_REV_60 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_Lift * 3.1415);
    private static final double DRIVE_SPEED = 0.75;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "Af33ubD/////AAABmZrw69bsukgitaZjU3qd+GgiLcfzvKbbEy92WSwqo1mIjB4OHY/nm5x1tMOf2flMwKepBsnohxy1jNnfUyjkwEvmMchNupexRWSMK7vw7nGT66f1AqGpdHdJZvzvxOAWHlX1DLEOMEyOvbsCcAjvtU2BND5QFLacoYyChBsMoQTt+LI3i+aPkEgZ+YEhFJbTQUQ807WXMWfpBBTI6xTvH1gy7zXI8zLvyje7Uap5vgKD7lWOUx04Xh7AI0Lmvyvw/DfcFs3V8hUgVOTUw3OgqvUS8hSxQv8Cqm74QHQSECuOFQVNwFfJGektlnNlZyKIGCvFwIHle/89bVSkhYh7hwcSgoQtTrtdVRn5n0KNoWPW";
    private static MediaPlayer mediaPlayer = null;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo StoneServoRight;  // close and open
    private Servo StoneServoLeft;   // close and open
    private DcMotor stoneLift;
    private DcMotor stoneTilt;
    private DcMotor autonPlatformLock;
    private Servo CapStoneServoLock;
    private Servo autonStoneServoRight;
    private Servo autonStoneServoLeft;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        stoneLift = hardwareMap.get(DcMotor.class, "motor5");
        stoneTilt = hardwareMap.get(DcMotor.class, "motor6");
        autonPlatformLock = hardwareMap.get(DcMotor.class, "motor7");
        StoneServoLeft = hardwareMap.servo.get("servo1");
        StoneServoRight = hardwareMap.servo.get("servo2");
        CapStoneServoLock = hardwareMap.servo.get("servo4");
        autonStoneServoLeft = hardwareMap.servo.get("servo5");
        autonStoneServoRight = hardwareMap.servo.get("servo6");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorLine");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeBlue");

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
        autonPlatformLock.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        StoneServoRight.setPosition(0.7);
        StoneServoLeft.setPosition(0.3);
        CapStoneServoLock.setPosition(0.0);
        autonStoneServoRight.setPosition(0.0);
        autonStoneServoLeft.setPosition(1.0);

        enableEncoders(); //enable the encoders
        runtime.reset();
        boolean runOnce = true;

        waitForStart();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (opModeIsActive() && runOnce) {
            move(-30, 0, 0, false);
            move(0, 16, 0, false);
            autonPlatformLock.setPower(-1.0);
            sleep(150);
            autonPlatformLock.setPower(0.0);
            move(30, 0, 0, false);
            autonPlatformLock.setPower(1.0);
            sleep(150);
            autonPlatformLock.setPower(0.0);
            move(0, -44, 0, false);

            runOnce = false;
        }
    }

    private void move(float strafeY, float strafeX, float turn, boolean turnSolo) {
        int leftFrontNew;
        int leftBackNew;
        int rightFrontNew;
        int rightBackNew;

        if (strafeY != 0) {
            //adding the distance to move in inches to current position
            leftFrontNew = leftFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH) - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH) - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH) + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH) + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);


            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);

        }
        if (strafeX != 0) {
            leftFrontNew = leftFront.getCurrentPosition() + (int) (1.25 * strafeX * COUNTS_PER_INCH) - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) (1.25 * strafeX * COUNTS_PER_INCH) - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() - (int) (1.25 * strafeX * COUNTS_PER_INCH) + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (1.25 * strafeX * COUNTS_PER_INCH) + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if (turn != 0 && turnSolo) {
            leftFrontNew = leftFront.getCurrentPosition() - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }


        while (leftFront.isBusy()) {
            telemetry.addData("LeftFontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("RightFontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
            telemetry.update();
            Thread.yield();
        }

        stopMotors();
        runWithEncoder(); //do we need this? I dont think so
    }

    private void movePos(int leftFrontNew, int leftBackNew, int rightFrontNew, int rightBackNew) {
        leftFront.setTargetPosition(leftFrontNew);
        leftBack.setTargetPosition(leftBackNew);
        rightFront.setTargetPosition(rightFrontNew);
        rightBack.setTargetPosition(rightBackNew);

        runToPositionEncoder(); //look out moving this to
        leftFront.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        rightBack.setPower(DRIVE_SPEED);
    }

    private void enableEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
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
    }

    public void runWithEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}