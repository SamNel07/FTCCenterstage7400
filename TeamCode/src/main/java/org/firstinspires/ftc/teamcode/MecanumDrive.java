package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Blinker;



/**
 * This is an example minimal implementation of the Mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="MecanumDrive")
public class MecanumDrive extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor lifter = null;
    private Servo plane = null;
    private Servo pl = null;
    private RevBlinkinLedDriver blinkin = null;

    private CRServo intake = null;


    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        //front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right  = hardwareMap.get(DcMotor.class, "front_right");

        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        //back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right   = hardwareMap.get(DcMotor.class, "back_right");


        //lifter   = hardwareMap.get(DcMotor.class, "lifter");
        //lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        plane = hardwareMap.get(Servo.class, "plane");
        pl = hardwareMap.get(Servo.class,"pl");
        intake = hardwareMap.get(CRServo.class, "intake");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor = hardwareMap.get(DcMotor.class, "lifter");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor.setDirection(DcMotor.Direction.REVERSE);

    }

    public DcMotor motor;
    public boolean buttonOperated = false;
    public ElapsedTime debouncer = null;

    public void set(int counts)
    {
        motor.setTargetPosition(counts);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(-0.8);
    }

    public void release()
    {
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    {
        debouncer = new ElapsedTime();
        debouncer.reset();


    }








    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).


        double drive  = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;
        double twist  = gamepad1.right_stick_y;




        /*if(gamepad2.dpad_up){
       lifter.setPower(0.8);
            }
else if (gamepad2.dpad_down){
       lifter.setPower(-0.8);
            }
else {
       lifter.setPower(0);

}*/
        //lifter.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        if (gamepad2.a && (debouncer.milliseconds() > 500))
        {
            if (buttonOperated)
            {
                //motor.setMode(DcMotor.RunMode.RUN)
                release();
            }

            buttonOperated = !buttonOperated;
            debouncer.reset();
        }

        if (buttonOperated)
        {
            if (gamepad2.x)
            {
                debouncer.reset();
                set(-2530);
            }

            else if (gamepad2.y)
            {
                debouncer.reset();
                //set(-2630);
                set(-2000);
            }

            else if (gamepad2.b)
            {
                debouncer.reset();
                set(0);
            }
      
        } else if (((motor.getCurrentPosition())<-2900) && (gamepad2.left_trigger - gamepad2.right_trigger) < 0) {
            motor.setPower(0);
        }
        else {
            motor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        }



        telemetry.addData("Lifter", motor.getCurrentPosition());
        telemetry.update();

        if(gamepad1.dpad_up){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        } else if (gamepad1.dpad_down) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if (gamepad1.dpad_left) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else if (gamepad1.dpad_right) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }


        if(gamepad1.y) {
            // move to 0 degrees.
            plane.setPosition(1);




        } else if (gamepad1.right_bumper) {
            // move to 90 degrees.
            plane.setPosition(0);


        }

        /*if(gamepad2.left_bumper) {
            pl.setPower(0.1);
        } else if (gamepad2.right_bumper) {
            pl.setPower(-0.1);
        } else if (gamepad2.y) {
            pl.setPower(-0.05);
        } else  {
            pl.setPower(0);
        }*/

        if(gamepad2.dpad_up) {
            intake.setPower(1);
        } else if (gamepad2.dpad_down) {
            intake.setPower(-0.3);
        } else if (gamepad2.dpad_right){
            intake.setPower(0);
        }




        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.


        double[] speeds = {
                (drive + -strafe + twist),
                (drive - strafe - twist),
                (drive - -strafe + twist),
                (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        if (gamepad1.right_trigger > 0.1) {
            front_left.setPower(speeds[0]*(0.25));
            front_right.setPower(speeds[1]*(0.25));
            back_left.setPower(speeds[2]*(0.25));
            back_right.setPower(speeds[3]*(0.25));
        } else {

            front_left.setPower(speeds[0]);
            front_right.setPower(speeds[1]);
            back_left.setPower(speeds[2]);
            back_right.setPower(speeds[3]);
        }
    }

}

