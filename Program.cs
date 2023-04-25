using CTRE.Phoenix;
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
        // Constants
        private const int ThreadLoopTime = 20;  // loop time in msec
        private const int SignalLightMax = 25;  // number of loops for toggling RSL

        // Create gamepad instance
        private const int BtnX = 1;             // X button
        private const int BtnA = 2;             // A button
        private const int BtnB = 3;             // B button
        private const int BtnY = 4;             // Y button
        private const int BtnLeftBumper = 5;    // Left Bumper
        private const int BtnRightBumper = 6;   // Right Bumper
        private const int BtnLeftTrigger = 7;   // Left Trigger
        private const int BtnRightTrigger = 8;  // Right Trigger
        private const int BtnBack = 9;          // Back button
        private const int BtnStart = 10;        // Start button
        private const int BtnLeftStick = 11;    // Left joystick press
        private const int BtnRightStick = 12;   // Right joystick press

        private const int AxisLeftStickX = 0;   // Left joystick X direction
        private const int AxisLeftStickY = 1;   // Left joystick Y direction
        private const int AxisRightStickX = 2;  // Right joystick X direction
        private const int AxisRightStickY = 5;  // Right joystick Y direction

        // TODO: DPAD directions?

        // Constants - PCM ports
        private const int PcmPortSignalLight = 7;   // Robot signal light port

        // Static objects
        private static LogitechGamepad _gamepad = new LogitechGamepad(UsbHostDevice.GetInstance(), 0);

        // Create four drive talons and PCM on CAN bus
        private static TalonSRX _leftFrnt = new TalonSRX(1);
        private static TalonSRX _leftRear = new TalonSRX(2);
        private static TalonSRX _rghtFrnt = new TalonSRX(3);
        private static TalonSRX _rghtRear = new TalonSRX(4);

        private static TalonSRX _wrist = new TalonSRX(6);

        private static PneumaticControlModule _pcm = new PneumaticControlModule(0);

        // Global variables
        private static bool _enabled = false;
        private static bool _enableBtnWasDown = false;
        private static DateTime _startTime;
        private static DateTime _enabledTime;
        private static int _signalLightCount = 0;

        //*********************************************************************
        //
        //  Main program
        //
        public static void Main()
        {
            // Get the start time for tracking total on time
            _startTime = DateTime.Now;

            // Invert all motor directions to match installation
            _rghtFrnt.SetInverted(true);
            _rghtRear.SetInverted(true);
            _leftFrnt.SetInverted(true);
            _leftRear.SetInverted(true);

            // TODO: Initialize PCM and enable compressor (if needed) Compressor may be automatic

            //
            // Main loop (forever)
            //
            while (true)
            {
                HandleEnabledState();           // check enable button and process it

                HandleDrive();                  // use joysticks to run drive motors

                HandleWrist();                  // use a joystick axis or dPad to move the wrist

                HandleButtons();                // read buttons and control the firing solenoids

                DebugController();              // temporary logging code for printing inputs

                Thread.Sleep(ThreadLoopTime);   // this provides the loop pacing at 20msec
            }
        }

        //*********************************************************************
        //
        //  Process enabled state operation of robot
        //
        private static void SetSignalLight(bool onState)
        {
            _pcm.SetSolenoidOutput(_pcmPortSignalLight, onState);
        }

        private static void HandleEnabledState()
        {
            if (_gamepad.GetConnectedStatus() != CTRE.Phoenix.UsbDeviceConnection.Connected)
                _enabled = false;

            // keep feeding watchdog to enable motors
            if (_enabled)
            {
                // this lets the drive motors, wrist motor, and PCM take commands
                Watchdog.Feed();

                // time out and disable after 3 minutes
                //  if (DateTime.Now.Ticks - _enabledTime.Ticks > 180)
                //      _enabled = false;

                if (_signalLightCount++ == SignalLightMax)
                {
                    if (_pcm.GetSolenoidOutput(_pcmPortSignalLight))
                        _pcm.SetSolenoidOutput(_pcmPortSignalLight, false);
                    else
                        _pcm.SetSolenoidOutput(_pcmPortSignalLight, true);
                    _signalLightCount = 0;
                }
            }
            else
                SetSignalLight(true);
        }

        //*********************************************************************
        //
        //  Deadband the joystick input
        //      If value is within +/-deadband range of center, clear it.
        //
        private static void Deadband(ref float value)
        {
            double deadband = 0.10;

            if (value < -deadband)
                value = (value + deadband) / (1.0 - deadband);  /* outside of deadband */
            else if (value > +deadband)
                value = (value - deadband) / (1.0 - deadband);  /* outside of deadband */
            else
                value = 0;                                      /* within deadband so zero it */
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
        private static void HandleDrive()
        {
            float x = _gamepad.GetAxis(AxisLeftStickX);      // left x: Positive is strafe-right, negative is strafe-left
            float y = -1 * _gamepad.GetAxis(AxisLeftStickY); // left y: Positive is forward, negative is reverse
            float turn = _gamepad.GetAxis(AxisRightStickX);     // right x: Positive is turn-right, negative is turn-left

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
            // _gamepad.GetBtn(POV);

            // derive an intended wrist angle
            // _wrist.getSelectedSensor(...);

            // Move wrist motor up/down
            if (_gamepad.GetButton(BtnLeftTrigger))
            {
                //_wrist.Set(ControlMode.PercentOutput, 0.5);
            }
            else (_gamepad.GetButton(BtnRightTrigger)
            {
                //_wrist.Set(ControlMode.PercentOutput, -0.5);
            }
            else
            {
                // _wrist.Set(TalonSRXControlMode.Position, wrist_angle);
            }
        }

        //*********************************************************************
        //
        //  Get gamepad buttos and handle actions needed
        //
        private static void HandleButtons()
        {
            // Enable button is pressed, enable and capture start time
            if (_gamepad.GetButton(BtnStart))
                _enableBtnWasDown = true;
            else if (_enableBtnWasDown)
            {
                _enabled = !_enabled;
                _enableBtnWasDown = false;
                if (_enabled)
                    _enabledTime = DateTime.Now;

                Debug.Print("BB-2 is now: " + ((_enabled) ? "ENABLED" : "DISABLED") + " at " + DateTime.Now());
            }

            // Fire PCM solenoids based on button input
            _pcm.SetSolenoidOutput(0, _gamepad.GetButton(BtnX));            // solenoid valve 1
            _pcm.SetSolenoidOutput(1, _gamepad.GetButton(BtnA));            // solenoid valve 2
            _pcm.SetSolenoidOutput(2, _gamepad.GetButton(BtnB));            // solenoid valve 3
            _pcm.SetSolenoidOutput(3, _gamepad.GetButton(BtnY));            // solenoid valve 4
            _pcm.SetSolenoidOutput(4, _gamepad.GetButton(BtnLeftBumper));   // solenoid valve 5
            _pcm.SetSolenoidOutput(5, _gamepad.GetButton(BtnRightBumper));  // solenoid valve 6            
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

                // Print to console so we can debug them
                Debug.Print("a0:" + axis0 + " a1:" + axis1 + " a2:" + axis2 + " a3:" + axis3 + " a4:" + axis4 + " a5:" + axis5 +
                            " b1:" + btn1 + " b2:" + btn2 + " b3:" + btn3 + " b4:" + btn4 + " b5:" + btn5 + " b6:" + btn6 +
                            " b7:" + btn7 + " b8:" + btn8 + " b9:" + btn9 + " b10:" + btn10 + " b11:" + btn11 + " b12:" + btn12 +
                            " b13" + btn13 + " b14:" + btn14 + " b15:" + btn15 + " b16:" + btn16);
            }
        }
    }
}
