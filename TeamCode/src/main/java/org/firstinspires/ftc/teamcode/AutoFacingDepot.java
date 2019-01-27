/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoFacingDepot", group = "Concept")
public class AutoFacingDepot extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //private static final PushbotAutoEncoder_Routes autoroutes = new PushbotAutoEncoder_Routes(Class.t);
    //Siddh Code
    public DcMotor leftDrive   = null;
    public DcMotor  rightDrive  = null;
    //public DcMotor  leftDrive2  = null;
    //public DcMotor  rightDrive2 = null;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    static final private double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final private double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final private double     WHEEL_DIAMETER_INCHES   = 8.0 ;     // For figuring circumference
    static final private double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final private double     DRIVE_SPEED             = 0.3;
    static final private double     TURN_SPEED              = 0.5;
    static final int                CYCLE_MS                = 2000;

    //end of Siddh code
    Servo servo;
    Servo marker;



    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AeUYNQD/////AAABmb0xGMnK6U+CuYc8ykzDmMBT12F33gTn667RqJbH/x99uxk0JpxzCzWarMRedJsfHtV1KskBtWgT0x3basLIhaI/g6znRRhnoHN2035GL9qyIC7ux0ZCdNuzOFzpYqDvtpEAxvK8y0Mcb4lFsz8bW8jdYYBEcE4SgyksTUMMEeOPh2r+ZDREbPPlymwvV6t/v/iXcgseA1XsM5kezfqosddyoEbEW/IX53vzRLdvZnltmWNrgD/Ef/kRIYztGJoY687DbsYQI2uvKIbb48VuWoMm8eCsbwP4Ows6TwzgrHXCKXVeeRbgqY2J2uvRfvGt1Qa1Z8epPA8pmU0h8xwm/oafU1RJeoKUrANdoJVcxuWM";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();


        servo = hardwareMap.get(Servo.class, "servo_cam");
        marker = hardwareMap.get(Servo.class, "marker_servo");





        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */





int i = 0;
        //autoroutes.init();
        //autoroutes.routesInit();

        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {

                if (i == 0){
                routesInit();
                encoderDrive(DRIVE_SPEED,  8,8,3.0);
                sleep(3000);
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                i++;
                }
                telemetry.addData("Hello!!", "Now running to go forward");

                // Exit when x becomes greater than 4

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                              //autoroutes.init();
                              //autoroutes.routesInit();
                              //autoroutes.facingDepot_middle();
                              middle();
                              telemetry.addData("Sorry!", "Called FaceDep");

                          }
                          /*if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                              servo.setPosition(0);
                              telemetry.addData("Hello!", "Just Moving the servo");
                              break;



                          }*/
                          else {
                              servo.setPosition(0.2);
                              if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                                  //checks if the second one is gold or silver
                                  faceleft();
                              }
                              else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                                  // if silver then will call the third gold.
                                  faceright();
                                  telemetry.addData("Sorry!", "Called FaceRight");
                              }
                            }

                          }



                      telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void middle() {
        routesInit();
        facingDepot_middle();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
    }
    public void faceleft() {
        routesInit();
        facingDepot_left();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
    }

    public void faceright(){
        routesInit();
        facingDepot_right();
    }


    public void routesInit(){
        leftDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightDrive = hardwareMap.get(DcMotor.class, "front_right");
        //leftDrive2 = hardwareMap.get(DcMotor.class, "back_left");
        //rightDrive2 = hardwareMap.get(DcMotor.class, "back_right");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //leftDrive2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightDrive2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //leftDrive2.setPower(0);
        //rightDrive2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
    }

    public void facingDepot_middle() {
        encoderDrive(DRIVE_SPEED,  45,45,10.0);  // go forward and hit gold mineral while dragging it into the depot and dropping the team marker
        marker.setPosition(0.7);
        sleep(CYCLE_MS);
        marker.setPosition(0.1);
        encoderDrive(DRIVE_SPEED, -6, -6, 3.0);
        encoderDrive(TURN_SPEED, 11, -11, 4.0);  // back up turning right
        encoderDrive(DRIVE_SPEED,   6, 6, 4.0);  // go forward
        encoderDrive(TURN_SPEED, 3.75, -3.75, 3.0);  //turn right
        //encoderDrive(DRIVE_SPEED, 6, 6, 4.0);
        //encoderDrive(TURN_SPEED, 1, -1, 2.0);
        encoderDrive(DRIVE_SPEED*2,   54, 54, 5.0);//go forward
        //encoderDrive(TURN_SPEED, 2.25, -2.25,5.0);//small adjustment to the right
        //encoderDrive(DRIVE_SPEED*2, 60, 60, 15.0);//go forward and park in crater
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    private void facingDepot_left() {
        encoderDrive(DRIVE_SPEED, 10, 10, 4.0); // go forward
        encoderDrive(TURN_SPEED, -5, 5, 5.0); // turn left
        encoderDrive(DRIVE_SPEED, 18, 18, 5.0); // go forward
        encoderDrive(TURN_SPEED, 9, -9, 5.0);//turn right
        encoderDrive(DRIVE_SPEED, 13, 13, 4.0); // go forward
        marker.setPosition(0.7);
        sleep(CYCLE_MS);
        marker.setPosition(0.1);
        encoderDrive(TURN_SPEED, 5, -5, 5.0);//turn right
        encoderDrive(DRIVE_SPEED, 9, 9, 4.0); // go forward
        encoderDrive(TURN_SPEED, 6, -6, 5.0); // turn right
        encoderDrive(DRIVE_SPEED * 2, 170, 170, 4.0); //go to crater


    }
    private void facingDepot_right(){

    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            //leftDrive2.setTargetPosition(newLeftTarget);
            //rightDrive2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            //leftDrive2.setPower(Math.abs(speed));
            //rightDrive2.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                //leftDrive2.getCurrentPosition(),
                //rightDrive2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            //leftDrive2.setPower(0);
            //rightDrive2.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            sleep(2500);   // optional pause after each move
        }
    }

}
