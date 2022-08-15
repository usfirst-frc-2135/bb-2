using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.Text;
using System.Threading;

namespace HERO_Mecanum_Drive_Example1
{
    public class 
        
        Program
    {
        /* create a talon */
        static TalonSRX leftFrnt = new TalonSRX(1);
        static TalonSRX leftRear = new TalonSRX(2);
        static TalonSRX rghtFrnt = new TalonSRX(3);
        static TalonSRX rghtRear = new TalonSRX(4);

        static PneumaticControlModule pcm = new PneumaticControlModule(0);

        static GameController _gamepad = null;

        public static void Main()
        {
            //int count = 0;

            /* loop forever */
            while (true)
            {
                /* keep feeding watchdog to enable motors */
                Watchdog.Feed();

                Drive();

                /*
                if (++count % 50 == 0)
                {
                    if (pcm.GetSolenoidOutput(0))
                        pcm.SetSolenoidOutput(0, false);
                    else
                        pcm.SetSolenoidOutput(0, true);
                }
                */

                Thread.Sleep(20);
            }
        }
        /**
         * If value is within 10% of center, clear it.
         * @param [out] floating point value to deadband.
         */
        static void Deadband(ref float value)
        {
            if (value < -0.10)
            {
                /* outside of deadband */
            }
            else if (value > +0.10)
            {
                /* outside of deadband */
            }
            else
            {
                /* within 10% so zero it */
                value = 0;
            }
        }
        /**
         * Nomalize the vector sum of mecanum math.  Some prefer to 
         * scale from the max possible value to '1'.  Others
         * prefer to simply cut off if the sum exceeds '1'.
         */
        static void Normalize(ref float toNormalize)
        {
            if (toNormalize > 1)
            {
                toNormalize = 1;
            }
            else if (toNormalize < -1)
            {
                toNormalize = -1;
            }
            else
            {
                /* nothing to do */
            }
        }
        static void Drive()
        {
            if (null == _gamepad)
                _gamepad = new GameController(UsbHostDevice.GetInstance());

            float strafe = -1* _gamepad.GetAxis(0);      // Positive is strafe-right, negative is strafe-left
            float turn = -1 * _gamepad.GetAxis(2);       // Positive is turn-right, negative is turn-left  
            float throttle = _gamepad.GetAxis(1);   // Positive is forward, negative is reverse

            Deadband(ref strafe);
            Deadband(ref turn);
            Deadband(ref throttle);

            float leftFrnt_throt = turn + strafe + throttle; // left front moves positive for forward, strafe-right, turn-right
            float leftRear_throt = turn - strafe + throttle; // left rear moves positive for forward, strafe-left, turn-right
            float rghtFrnt_throt = turn - strafe - throttle; // right front moves positive for forward, strafe-left, turn-left
            float rghtRear_throt = turn + strafe - throttle; // right rear moves positive for forward, strafe-right, turn-left

            /* normalize here, there a many way to accomplish this, this is a simple solution */
            Normalize(ref leftFrnt_throt);
            Normalize(ref leftRear_throt);
            Normalize(ref rghtFrnt_throt);
            Normalize(ref rghtRear_throt);

            /* everything up until this point assumes positive spins motor so that robot moves forward.
                But typically one side of the robot has to drive negative (red LED) to move robor forward.
                Assuming the left-side has to be negative to move robot forward, flip the left side */
            rghtFrnt_throt *= -1;
            rghtRear_throt *= -1;

            leftFrnt.Set(ControlMode.PercentOutput, leftFrnt_throt);
            leftRear.Set(ControlMode.PercentOutput, leftRear_throt);
            rghtFrnt.Set(ControlMode.PercentOutput, rghtFrnt_throt);
            rghtRear.Set(ControlMode.PercentOutput, rghtRear_throt);

            Debug.Print("  strafe:" + strafe + "  turn:" + turn + "  f/r:" + throttle);
        }
    }
}