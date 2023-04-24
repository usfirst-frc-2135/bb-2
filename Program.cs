﻿using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.Text;
using System.Threading;

//*****************************************************************************
//
//  BB-2: Presentation Invasion 2135 - T-shirt shooter
//
namespace BB_2
{
    public class Program
    {
        // Create four drive talons and PCM
        private static TalonSRX _leftFrnt = new TalonSRX(1);
        private static TalonSRX _leftRear = new TalonSRX(2);
        private static TalonSRX _rghtFrnt = new TalonSRX(3);
        private static TalonSRX _rghtRear = new TalonSRX(4);
        private static TalonSRX _wrist = new TalonSRX(6);
        private static PneumaticControlModule _pcm = new PneumaticControlModule(0);

        // Create gamepad instance
        private static LogitechGamepad _gamepad = null;    // Investigate if this is a better gamepad object

        private static CANifier _canifier = new CANifier(0);

        // Global variables
        private static bool _enabled = false;
        private static bool _enableBtnWasDown = false;
        private static DateTime _startTime;
        private static DateTime _enableTime;

        private static int _signalLightCount = 0;
        private static int _kPCMSignalLightPort = 7;

        //*********************************************************************
        //
        //  Main program
        //
        public static void Main()
        {
            // Get the start time for tracking total on time
            _startTime = DateTime.Now;

            // Create the gamepad object
            if (null == _gamepad)
                _gamepad = new LogitechGamepad(UsbHostDevice.GetInstance(), 0);

            // Invert all motor directions to match installation
            _rghtFrnt.SetInverted(true);
            _rghtRear.SetInverted(true);
            _leftFrnt.SetInverted(true);
            _leftRear.SetInverted(true);

            // Initialize PCM and enable compressor
            // (if needed) Compressor may be automatic


            //
            // Main loop (forever)
            //
            while (true)
            {
                // keep feeding watchdog to enable motors
                if (_enabled)
                    Watchdog.Feed();

                // use joysticks to run drive motors
                Drive();

                // use a joystick axis or dPad to move the wrist
                HandleWrist();

                // read buttons and control the firing solenoids
                HandleButtons();

                // temporary logging code for printing inputs
                DebugController();

                // signal light flashes when enabled and solid when disabled 
                HandleSignalLight();

                // this provides the loop pacing at 20msec
                Thread.Sleep(20);
            }
        }

        //*********************************************************************
        //
        //  Deadband the joystick input
        //      If value is within 10% of center, clear it.
        //
        private static void Deadband(ref float value)
        {
            if (value < -0.10)
            { /* outside of deadband */ }
            else if (value > +0.10)
            { /* outside of deadband */ }
            else
            { /* within 10% so zero it */
                value = 0;
            }
        }

        //*********************************************************************
        //
        //  Nomalize the vector sum of mecanum math.
        //      Some prefer to  scale from the max possible value to '1'.
        //      Others prefer to simply cut off if the sum exceeds '1'.
        //
        private static void Normalize(ref float toNormalize)
        {
            if (toNormalize > 1)
                toNormalize = 1;
            else if (toNormalize < -1)
                toNormalize = -1;
            else
            { /* nothing to do */ }
        }

        //*********************************************************************
        //
        //  Run drive motors from joystick inputs
        //      X axis left - strafe left/right
        //      Y axis left - forward/back (joystick axis Y is negative forward)
        //      X axis right - turn left/right
        //
        private static void Drive()
        {
            float x    = _gamepad.GetAxis(0);      // left x: Positive is strafe-right, negative is strafe-left
            float y    = -1 * _gamepad.GetAxis(1); // left y: Positive is forward, negative is reverse
            float turn = _gamepad.GetAxis(2);      // right x: Positive is turn-right, negative is turn-left

            Deadband(ref x);
            Deadband(ref y);
            Deadband(ref turn);

            float _leftFrnt_throt = y + x + turn;   // left front moves positive for forward, strafe-right, turn-right
            float _leftRear_throt = y - x + turn;   // left rear moves positive for forward, strafe-left, turn-right
            float _rghtFrnt_throt = y - x - turn;   // right front moves positive for forward, strafe-left, turn-left
            float _rghtRear_throt = y + x - turn;   // right rear moves positive for forward, strafe-right, turn-left

            // normalize here, there a many way to accomplish this, this is a simple solution
            Normalize(ref _leftFrnt_throt);
            Normalize(ref _leftRear_throt);
            Normalize(ref _rghtFrnt_throt);
            Normalize(ref _rghtRear_throt);

            // Control the motors for mecanum drive operation
            _leftFrnt.Set(ControlMode.PercentOutput, _leftFrnt_throt);
            _leftRear.Set(ControlMode.PercentOutput, _leftRear_throt);
            _rghtFrnt.Set(ControlMode.PercentOutput, _rghtFrnt_throt);
            _rghtRear.Set(ControlMode.PercentOutput, _rghtRear_throt);
        }

        //*********************************************************************
        //
        //  Get dpad input and operate wrist
        //
        private static void HandleWrist()
        {
            // Get wrist axis or dPad from gamepad
            bool armMoveDown = _gamepad.GetButton(7);  // left trigger
            bool armMoveUp = _gamepad.GetButton(8);    // right trigger


            // _gamepad.GetBtn(POV);
            // derive an intended wrist angle

            // Move wrist motor up/down
            if (_gamepad.GetButton(7))
            {
                //_wrist.Set(ControlMode.PercentOutput, )
            }

            // _wrist.Set(TalonSRXControlMode.Position, wrist_angle);
        }

        //*********************************************************************
        //
        //  Get gamepad buttos and handle actions needed
        //
        private static void HandleButtons()
        {
            // Get the shooting buttons
            // Fire PCM solenoids based on button input
            bool btnX = _gamepad.GetButton(1);         // x
            bool btnA = _gamepad.GetButton(2);         // a
            bool btnB = _gamepad.GetButton(3);         // b
            bool btnY = _gamepad.GetButton(4);         // y
            bool btnLBumper = _gamepad.GetButton(5);   // left bumper
            bool btnRBumper = _gamepad.GetButton(6);   // right bumper
            bool btnLTrigger = _gamepad.GetButton(7);  // left trigger
            bool btnRTrigger = _gamepad.GetButton(8);  // right trigger
            bool btnBack = _gamepad.GetButton(9);      // back
            bool btnStart = _gamepad.GetButton(10);    // start
            bool btnLStick = _gamepad.GetButton(11);   // left jstick
            bool btnRStick = _gamepad.GetButton(12);   // right jstick

            _pcm.SetSolenoidOutput(0, btnX);           // solenoid valve 1
            _pcm.SetSolenoidOutput(1, btnA);           // solenoid valve 2
            _pcm.SetSolenoidOutput(2, btnB);           // solenoid valve 3
            _pcm.SetSolenoidOutput(3, btnY);           // solenoid valve 4
            _pcm.SetSolenoidOutput(4, btnLBumper);     // solenoid valve 5
            _pcm.SetSolenoidOutput(5, btnRBumper);     // solenoid valve 6

            

            // Enable button is pressed, enable and capture start time
            if (btnStart)
                _enableBtnWasDown = true;
            else if (_enableBtnWasDown)
            {
                _enabled = !_enabled;
                Debug.Print("BB-2 is now: " + ((_enabled) ? "_enabled" : "DISABLED"));

                _enableBtnWasDown = false;
                if (_enabled)
                    _enableTime = DateTime.Now;
            }

            // If _enabled, time out and disable after 3 minutes
            if (_enabled)
            {
                //        if (DateTime.Now.Ticks - _enableTime.Ticks > 180)
                //          _enabled = false;
            }
        } 

        //*********************************************************************
        //
        //  Debug gamepad axes and buttons
        //
        private static void DebugController()
        {
            if (true)
            {
                // Get all axis and buttons
                float axis0 = _gamepad.GetAxis(0);      // left jstick x -1.0 to 1.0
                float axis1 = _gamepad.GetAxis(1);      // left jstick y 1.0 to -1.0
                float axis2 = _gamepad.GetAxis(2);      // right jstick x -1.0 to 1.0
                float axis3 = _gamepad.GetAxis(3);
                float axis4 = _gamepad.GetAxis(4);
                float axis5 = _gamepad.GetAxis(5);      // right jstick y 1.0 to -1.0

                bool btn1 = _gamepad.GetButton(1);       // x
                bool btn2 = _gamepad.GetButton(2);       // a
                bool btn3 = _gamepad.GetButton(3);       // b
                bool btn4 = _gamepad.GetButton(4);       // y
                bool btn5 = _gamepad.GetButton(5);       // left bumper
                bool btn6 = _gamepad.GetButton(6);       // right bumper
                bool btn7 = _gamepad.GetButton(7);       // left trigger
                bool btn8 = _gamepad.GetButton(8);       // right trigger
                bool btn9 = _gamepad.GetButton(9);       // back
                bool btn10 = _gamepad.GetButton(10);     // start
                bool btn11 = _gamepad.GetButton(11);     // left jstick
                bool btn12 = _gamepad.GetButton(12);     // right jstick
                bool btn13 = _gamepad.GetButton(13);
                bool btn14 = _gamepad.GetButton(14);
                bool btn15 = _gamepad.GetButton(15);
                bool btn16 = _gamepad.GetButton(16);

                bool Down = _gamepad.GetButton(7);
                bool Up = _gamepad.GetButton(8);

                // Print to console so we can debug them
                Debug.Print("a0: " + axis0 + " a1:" + axis1 + " a2:" + axis2 +
                    " a3:" + axis3 + " a4:" + axis4 + " a5:" + axis5 +
                    " b1:" + btn1 + " b2:" + btn2 + " b3:" + btn3 + " b4:" + btn4 + 
                    " b5:" + btn5 + " b6:" + btn6 + " b7:" + btn7 + " b8:" + btn8 + 
                    " b9:" + btn9 + " b10:" + btn10 + " b11:" + btn11 + " b12:" + btn12 +
                    " b13" + btn13 + " b14:" + btn14 + " b15:" + btn15 + " b16:" + btn16 + " down:" + Down + " up:" + Up);

                _canifier.SetLEDOutput(100, 0);
            }
        }
        private static void FlashSignalLight()
        {
            _signalLightCount++;
            if (_signalLightCount == 25)
            {
                //turn on
                if (_pcm.GetSolenoidOutput(_kPCMSignalLightPort))
                    _pcm.SetSolenoidOutput(_kPCMSignalLightPort, false);
                //off light
                else
                    _pcm.SetSolenoidOutput(_kPCMSignalLightPort, true);
                _signalLightCount = 0;
            }
        }

        private static void SetSignalLight(bool onState)
        {
            _pcm.SetSolenoidOutput(_kPCMSignalLightPort, onState);
        }

        private static void HandleSignalLight()
        {
            if (_enabled)
                FlashSignalLight();
            else
                SetSignalLight(true);
        }
    }
}
