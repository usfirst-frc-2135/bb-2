using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.LED;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.Threading;

//*****************************************************************************
//
//  BB-2: Presentation Invasion 2135 - T-shirt shooter
//
namespace BB_2
{
  public class Program
  {
    // Constants - System
    private const int ThreadLoopTime = 10;  // loop time in msec

    // Constants - Create gamepad instance
    private const int BtnX = 1;             // X button             - fire barrel 4
    private const int BtnA = 2;             // A button             - fire barrel 3
    private const int BtnB = 3;             // B button             - fire barrel 2
    private const int BtnY = 4;             // Y button             - fire barrel 1
    private const int BtnLeftBumper = 5;    // Left Bumper          - wrist up
    private const int BtnLeftTrigger = 7;   // Left Trigger         - wrist down
    private const int BtnRightBumper = 6;   // Right Bumper         - fire barrel 5
    private const int BtnRightTrigger = 8;  // Right Trigger        - fire barrel 6
    // private const int BtnBack = 9;          // Back button          - unused
    private const int BtnStart = 10;        // Start button         - enable robot
    // private const int BtnLeftStick = 11;    // Left joystick press  - unused
    // private const int BtnRightStick = 12;   // Right joystick press - unused

    private const int AxisLeftStickX = 0;   // Left joystick X direction    - strafe
    private const int AxisLeftStickY = 1;   // Left joystick Y direction    - forward/reverse
    private const int AxisRightStickX = 2;  // Right joystick X direction   - rotation
    // private const int AxisRightStickY = 5;  // Right joystick Y direction   - unused

    private const int NumButtons = 12;
    // private const int NumAxes = 6;

    // NOTE: DPAD does not work using getPOV() must use getAllValues()!
    private enum PovBtns                    // Raw values returned for POV DPad relative to compass direction
    {
      North = 0,
      NorthEast = 1,
      East = 2,
      SouthEast = 3,
      South = 4,
      SouthWest = 5,
      West = 6,
      NorthWest = 7,
      None = 8
    };


    // Mechanical constants
    private const int EncoderCountsPerRev = 4096;            // Wrist encoder counts for one shaft rotation
    private const float WristGearReduction = 62.0F / 18.0F;  // Wrist reduction in gear teeth
    private const float WristChainReduction = 30.0F / 12.0F; // Wrist reduction in chain sprocket teeth
    private const float WristGearRatio = WristGearReduction * WristChainReduction; // Wrist Gear Ratio combined
    private const float WristAngleMin = 0.0F;                // Wrist Angle Minimum for Soft Limit
    private const float WristAngleMax = 45.0F;               // Wrist Angle Maximum for Soft Limit
    private const float WristMoveSpeed = 0.2F;               // Wrist output power

    private const float ShooterValveOpenTime = 60.0F;        // Duration of shooter value open pulse

    // Color constants for CANdle
    private struct ColorGRB
    {
      public int g;
      public int r;
      public int b;
    };

    private static ColorGRB Red    = new ColorGRB { r = 255, g = 0,   b = 0 };
    private static ColorGRB Orange = new ColorGRB { r = 255, g = 48,  b = 0 };
    private static ColorGRB Green  = new ColorGRB { r = 0,   g = 255, b = 0 };
    private static ColorGRB Blue   = new ColorGRB { r = 0,   g = 0,   b = 255 };
    private static ColorGRB Purple = new ColorGRB { r = 128, g = 0,   b = 128 };
    private static ColorGRB White  = new ColorGRB { r = 255, g = 255, b = 255 };
    private static ColorGRB Off    = new ColorGRB { r = 0,   g = 0,   b = 0 };

    // Constants - PCM ports
    private const int ShooterNumValves = 6;

    // Constants - Joysticks
    private const float Deadband = 0.10F;   // Joystick deadband for driving

    // Constants - CANDle settings
    private const float Brightness = 0.75F; // CANDle brightness level
    private const int NumLeds = 38;         // CANDle total number of LEDs
    private const int OffsetLed = 0;        // CANDle offset of first LED
    private const float Speed = 0.5F;       // CANDle animation speed
    private const int WhiteValue = 0;       // CANDle white level
    private const int LedPeriodMs = 500;    // LED flashing period in msec

    // Static objects
    private static readonly LogitechGamepad _gamepad = new LogitechGamepad(UsbHostDevice.GetInstance(), 0);

    // Create four drive talons, PCM, and CANdle on CAN bus
    private static readonly TalonSRX _leftFrnt = new TalonSRX(1);
    private static readonly TalonSRX _leftRear = new TalonSRX(2);
    private static readonly TalonSRX _rghtFrnt = new TalonSRX(3);
    private static readonly TalonSRX _rghtRear = new TalonSRX(4);
    private static readonly TalonSRX _wrist = new TalonSRX(6);
    private static readonly CANdle _candle = new CANdle(0);
    private static readonly PneumaticControlModule _pcm = new PneumaticControlModule(0);

    // Global variables
    private static bool _enabled = false;
    private static DateTime _enabledTime;
    private static readonly Animation[] _animation = {
            new SingleFadeAnimation(Green.r, Green.g, Green.b, WhiteValue, Speed, NumLeds, OffsetLed), // Disabled animation
            new ColorFlowAnimation(Blue.r, Blue.g, Blue.b, WhiteValue, Speed, NumLeds, ColorFlowAnimation.ColorFlowDirection.Forward, OffsetLed),
            new FireAnimation(Brightness, Speed, NumLeds, 1.0F, 1.0F, false, OffsetLed),
            new LarsonAnimation(Red.r, Red.g, Red.b, WhiteValue, Speed, NumLeds, LarsonAnimation.LarsonBounceMode.Front, 4, OffsetLed),
            new RainbowAnimation(Brightness, Speed, NumLeds, false, OffsetLed),
            new RgbFadeAnimation(Brightness, Speed, NumLeds, OffsetLed),
            new StrobeAnimation(Orange.r, Orange.g, Orange.b, WhiteValue, Speed/10, NumLeds, OffsetLed),
            new TwinkleAnimation(White.r, White.g, White.b, WhiteValue, Speed, NumLeds, TwinkleAnimation.TwinklePercent.Percent64, OffsetLed),
            new TwinkleOffAnimation(Purple.r, Purple.g, Purple.b, WhiteValue, Speed, NumLeds, TwinkleOffAnimation.TwinkleOffPercent.Percent64, OffsetLed)
        };
    private static int _activeAnimation = 3; // Default animation for enabled state

    //*********************************************************************
    //*********************************************************************
    //
    //  Initialization
    //
    private static void ConfigDrive()
    {
      // Invert all motor directions to match mechanical gearbox installation
      _rghtFrnt.SetInverted(true);
      _rghtRear.SetInverted(true);
      _leftFrnt.SetInverted(true);
      _leftRear.SetInverted(true);
    }

    private static int WristDegreesToTalon(float degrees, float gearRatio)
    {
      return (int)((degrees / 360.0F) * EncoderCountsPerRev * gearRatio);
    }

    private static float WristTalonToDegrees(int counts, float gearRatio)
    {
      return ((float)counts / EncoderCountsPerRev) * 360.0F / gearRatio;
    }

    private static float GetWristDegrees()
    {
      return WristTalonToDegrees(_wrist.GetSelectedSensorPosition(), WristGearRatio);
    }

    private static void SetWristDegrees(float degrees)
    {
      _wrist.SetSelectedSensorPosition(WristDegreesToTalon(degrees, WristGearRatio));
    }

    private static void ConfigWrist()
    {
      // Match mechanical installation for gearbox and motor
      _wrist.SetInverted(true);
      _wrist.SetSensorPhase(false);
      _wrist.SetSelectedSensorPosition(0);
      _wrist.SetNeutralMode(NeutralMode.Brake);

      _wrist.ConfigReverseSoftLimitThreshold((int)WristDegreesToTalon(WristAngleMin, WristGearRatio));
      _wrist.ConfigReverseSoftLimitEnable(true);
      _wrist.ConfigForwardSoftLimitThreshold((int)WristDegreesToTalon(WristAngleMax, WristGearRatio));
      _wrist.ConfigForwardSoftLimitEnable(true);
    }

    private static void ConfigCANdle()
    {
      _candle.ConfigFactoryDefault();

      CANdleConfiguration configAll = new CANdleConfiguration
      {
        brightnessScalar = Brightness,      // Initialize LED brightness
        disableWhenLOS = false,             // Don't disable LEDs when "loss of signal"
        statusLedOffWhenActive = true,      // Turn off the CAN status LED when LEDs are set
        stripType = LEDStripType.GRB,       // Order of LEDs in extra strip
        v5Enabled = true,                   // Enable 5.0V output to power extra strip
        vBatOutputMode = VBatOutputMode.Off // Disable VBat (12.0V) since not used
      };
      _candle.ConfigAllSettings(configAll);

      _candle.ClearAnimation(0);
      _candle.Animate(_animation[0]);       // Disabled animation is zero
    }

    private static void ConfigValves()
    {
      // Initialize shooter control valves
      HandleShooterValvePulse(0, false);
      HandleShooterValvePulse(1, false);
      HandleShooterValvePulse(2, false);
      HandleShooterValvePulse(3, false);
      HandleShooterValvePulse(4, false);
      HandleShooterValvePulse(5, false);
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Deadband the joystick input
    //      If value is within +/-deadband range of center, clear it.
    //
    private static void StickDeadband(ref float value)
    {
      if (value < -Deadband)
        value = (value + Deadband) / (1.0F - Deadband);  /* outside of deadband, scale it */
      else if (value > +Deadband)
        value = (value - Deadband) / (1.0F - Deadband);  /* outside of deadband, scale it */
      else
        value = 0;                                       /* within deadband, zero it */
    }

    //*********************************************************************
    //
    //  Nomalize the vector sum of mecanum math.
    //      Some prefer to  scale from the max possible value to '1'.
    //      Others prefer to simply cut off if the sum exceeds '1'.
    //
    private static void DriveNormalize(ref float toNormalize)
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
      float x = _gamepad.GetAxis(AxisLeftStickX);         // left x: Positive is strafe-right, negative is strafe-left
      float y = -1.0F * _gamepad.GetAxis(AxisLeftStickY); // left y: Positive is forward, negative is reverse
      float turn = _gamepad.GetAxis(AxisRightStickX);     // right x: Positive is turn-right, negative is turn-left

      if (!_enabled)
      {
        x = 0.0F;
        y = 0.0F;
        turn = 0.0F;
      }

      StickDeadband(ref x);
      StickDeadband(ref y);
      StickDeadband(ref turn);

      float _leftFrnt_throt = y + x + turn;   // left front moves positive for forward, strafe-right, turn-right
      float _leftRear_throt = y - x + turn;   // left rear moves positive for forward, strafe-left, turn-right
      float _rghtFrnt_throt = y - x - turn;   // right front moves positive for forward, strafe-left, turn-left
      float _rghtRear_throt = y + x - turn;   // right rear moves positive for forward, strafe-right, turn-left

      // Normalize here (limit to range -1.0 .. 1.0)
      DriveNormalize(ref _leftFrnt_throt);
      DriveNormalize(ref _leftRear_throt);
      DriveNormalize(ref _rghtFrnt_throt);
      DriveNormalize(ref _rghtRear_throt);

      // Control the motors for mecanum drive operation
      _leftFrnt.Set(ControlMode.PercentOutput, _leftFrnt_throt);
      _leftRear.Set(ControlMode.PercentOutput, _leftRear_throt);
      _rghtFrnt.Set(ControlMode.PercentOutput, _rghtFrnt_throt);
      _rghtRear.Set(ControlMode.PercentOutput, _rghtRear_throt);
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Detect button pressed - return true only when first depressed
    //
    public static bool[] btnSave = new bool[NumButtons] { false, false, false, false, false, false, false, false, false, false, false, false };

    private static bool IsButtonPressed(uint buttonIdx)
    {
      bool result = false;
      bool btnState = _gamepad.GetButton(buttonIdx);

      if (btnState && !btnSave[buttonIdx])
        result = true;

      btnSave[buttonIdx] = btnState;

      return result;
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Get button input and operate wrist
    //
    private static bool IsPovHeld(PovBtns pov)
    {
      GameControllerValues _gpadAllValues = new GameControllerValues();

      _gamepad.GetAllValues(ref _gpadAllValues);

      return ((PovBtns)_gpadAllValues.pov == pov);
    }

    private static void HandleWristButtons()
    {
      float wristOutput = 0.0F;

      // Use wrist buttons for up and down to control wrist elevation
      if (_enabled)
      {
        if (IsPovHeld(PovBtns.North))
          wristOutput = WristMoveSpeed;
        else if (IsPovHeld(PovBtns.South))
          wristOutput = -WristMoveSpeed;
      }

      _wrist.Set(TalonSRXControlMode.PercentOutput, wristOutput);
      if (wristOutput != 0.0F)
      {
        int wristPosition = _wrist.GetSelectedSensorPosition();
        Debug.Print("Wrist: Moving " + ((wristOutput > 0.0) ? "UP" : "DDWN")
          + " - Position: " + wristPosition
          + " Degrees " + WristTalonToDegrees(wristPosition, WristGearRatio));
      }
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Handle Valve pulse timing (minValue means timer is not in use)
    //
    private static DateTime[] openTime = new DateTime[ShooterNumValves] { DateTime.MinValue, DateTime.MinValue, DateTime.MinValue, DateTime.MinValue, DateTime.MinValue, DateTime.MinValue };

    private static bool HandleShooterValvePulse(int valveIdx, bool startPulse)
    {
      bool valveOpen = false;

      if ((openTime[valveIdx] == DateTime.MinValue) && startPulse)
        openTime[valveIdx] = DateTime.Now;

      if ((openTime[valveIdx] != DateTime.MinValue) &&
          (DateTime.Now.Subtract(openTime[valveIdx]).Milliseconds < ShooterValveOpenTime))
        valveOpen = true;
      else
        openTime[valveIdx] = DateTime.MinValue;

      _pcm.SetSolenoidOutput(valveIdx, valveOpen);
      return valveOpen;
    }

    //*********************************************************************
    //*********************************************************************
    //
    //   Fire PCM solenoids based on button input - barrel numbers are clockwise from rear of robot
    //
    private static void HandleFiringButtons()
    {
      HandleShooterValvePulse(0, IsButtonPressed(BtnY));            // Barrel 1
      HandleShooterValvePulse(1, IsButtonPressed(BtnB));            // Barrel 2
      HandleShooterValvePulse(2, IsButtonPressed(BtnA));            // Barrel 3
      HandleShooterValvePulse(3, IsButtonPressed(BtnX));            // Barrel 4
      HandleShooterValvePulse(4, IsButtonPressed(BtnRightBumper));  // Barrel 5
      HandleShooterValvePulse(5, IsButtonPressed(BtnRightTrigger)); // Barrel 6
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Process CANDle selection
    //
    private static void HandleCANdleState()
    {
      int requestedAnimation = _activeAnimation; // Default request to current animation

      if (IsButtonPressed(BtnLeftBumper))
      {
        requestedAnimation += 1;
        if (requestedAnimation >= _animation.Length)
          requestedAnimation = 1;
      }
      else if (IsButtonPressed(BtnLeftTrigger))
      {
        requestedAnimation -= 1;
        if (requestedAnimation < 1)
          requestedAnimation = _animation.Length - 1;
      }

      if (_enabled && (requestedAnimation != _activeAnimation))
      {
        _activeAnimation = requestedAnimation;
        _candle.ClearAnimation(0);
        _candle.Animate(_animation[_activeAnimation]);
      }
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Detect start button and use it to enable the robot
    //
    private static void HandleEnabledState()
    {
      bool enableRequest = _enabled;  // Default request to current enable state
      TimeSpan onTime = DateTime.Now.Subtract(_enabledTime);

      // Enable button is pressed, request a state toggle
      if (IsButtonPressed(BtnStart))
        enableRequest = !_enabled;

      // Gamepad disconnect, disable
      if (_gamepad.GetConnectionStatus() != CTRE.Phoenix.UsbDeviceConnection.Connected)
        enableRequest = false;

      // Timeout if enabled for a long time
      if (_enabled && ((onTime.Minutes * 60 + onTime.Seconds) > 180))
          enableRequest = false;

      // If enabled request is a change from previous state, update state
      if (enableRequest != _enabled)
      {
        _enabled = enableRequest;

        _candle.ClearAnimation(0);
        if (_enabled)
        {
          _enabledTime = DateTime.Now;
          _candle.Animate(_animation[_activeAnimation]);
        }
        else
          _candle.Animate(_animation[0]);

        Debug.Print("BB-2 is now: " + ((_enabled) ? "ENABLED" : "DISABLED") + " at " + DateTime.Now + " seconds");
      }

      if (_enabled)
        Watchdog.Feed();
    }

    //*********************************************************************
    //*********************************************************************
    //
    //  Debug gamepad axes and buttons
    //
    private static int _debugPrintCount = 0;

    private static void DebugController()
    {
      if (++_debugPrintCount % 50 == 0)
      {
        // GameControllerValues gpadValues = new GameControllerValues();
        // _gamepad.GetAllValues(ref gpadValues);

        // float[] a = gpadValues.axes;
        // uint btns = gpadValues.btns;
        // int pov = gpadValues.pov;

        // Debug.Print("a0:" + a[0] + " a1:" + a[1] + " a2:" + a[2] + " a3:" + a[3] + " a4:" + a[4] + " a5:" + a[5] +
        //    " btns:" + btns + " pov:" + pov);

        // Get all axis and buttons
        // float axis0 = _gamepad.GetAxis(0);      // left jstick x -1.0 to 1.0
        // float axis1 = _gamepad.GetAxis(1);      // left jstick y 1.0 to -1.0
        // float axis2 = _gamepad.GetAxis(2);      // right jstick x -1.0 to 1.0
        // float axis3 = _gamepad.GetAxis(3);      // (doesn't work)
        // float axis4 = _gamepad.GetAxis(4);      // (doesn't work)
        // float axis5 = _gamepad.GetAxis(5);      // right jstick y 1.0 to -1.0

        // bool btn1 = _gamepad.GetButton(1);       // x
        // bool btn2 = _gamepad.GetButton(2);       // a
        // bool btn3 = _gamepad.GetButton(3);       // b
        // bool btn4 = _gamepad.GetButton(4);       // y
        // bool btn5 = _gamepad.GetButton(5);       // left bumper
        // bool btn6 = _gamepad.GetButton(6);       // right bumper
        // bool btn7 = _gamepad.GetButton(7);       // left trigger
        // bool btn8 = _gamepad.GetButton(8);       // right trigger
        // bool btn9 = _gamepad.GetButton(9);       // back
        // bool btn10 = _gamepad.GetButton(10);     // start
        // bool btn11 = _gamepad.GetButton(11);     // left jstick
        // bool btn12 = _gamepad.GetButton(12);     // right jstick
        // int pov0 = _gamepad.GetPov();            // POV (doesn't work!)

        // Print to console so we can debug them
        //    Debug.Print("a0:" + axis0 + " a1:" + axis1 + " a2:" + axis2 + " a3:" + axis3 + " a4:" + axis4 + " a5:" + axis5 +
        //                " b1:" + btn1 + " b2:" + btn2 + " b3:" + btn3 + " b4:" + btn4 + " b5:" + btn5 +
        //                " b6:" + btn6 + " b7:" + btn7 + " b8:" + btn8 + " b9:" + btn9 + " b10:" + btn10 +
        //                " b11:" + btn11 + " b12:" + btn12 + " pov:" + pov0);
      }
    }

    //*********************************************************************
    //
    //  Main program
    //
    public static void Main()
    {
      // Get the start time for tracking total on time
      ConfigDrive();
      ConfigWrist();
      ConfigValves();
      ConfigCANdle();

      // TODO: Initialize PCM and enable compressor (if needed) Compressor may be automatic

      //
      // Main loop (forever)
      //
      while (true)
      {
        HandleDrive();                  // use joysticks to run drive motors
        HandleWristButtons();           // handle buttons that control wrist elevation
        HandleFiringButtons();          // handle buttons that control the firing solenoids
        HandleCANdleState();            // control CANDel LED patterns
        HandleEnabledState();           // check enable state and process it
        DebugController();              // temporary logging code for printing inputs

        Thread.Sleep(ThreadLoopTime);   // this provides the loop pacing
      }
    }
  }
}
