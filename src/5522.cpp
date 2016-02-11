#include "WPILib.h"
#include <stdio.h>
#include <sys/time.h>
#include <AnalogGyro.h>

/**
 * This sample program shows how to control a motor using a joystick. In the operator
 * control part of the program, the joystick is read and the value is written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as range from
 * -1 to 1 making it easy to work together. The program also delays a short time in the loop
 * to allow other threads to run. This is generally a good idea, especially since the joystick
 * values are only transmitted from the Driver Station once every 20ms.
 */
class Robot : public SampleRobot {
	Joystick m_stick;
	Joystick m_stick1;
	Joystick m_stick2;
	AnalogGyro gyro;
	float Sol;
	float Sor;
	bool Sob12;
	bool Sob5;
	bool Sob6;
	bool Sob7;
	bool Sob8;
	bool sob11;
	int counter;
	int timer1;
	bool autoDriveMode;
	// The motor to control with the Joystick.
	// This uses a Talon speed controller; use the Victor or Jaguar classes for
	//   other speed controllers.
	Talon MLeft1;
	Talon MLeft2;
	Talon MRight1;
	Talon MRight2;
	Talon ShooterL;
	Talon ShooterR;
	timeval tm;
	timeval tm_last;
		// Solenoids to control with the joystick.
		// Solenoid corresponds to a single solenoid.
		Solenoid m_solenoid;
		// DoubleSolenoid corresponds to a double solenoid.
		DoubleSolenoid m_doubleSolenoid;
		// Update every 5milliseconds/0.005 seconds.
		const double kUpdatePeriod = 0.005;

		// Numbers of the buttons to use for triggering the solenoids.
		const int kSolenoidButton = 1;
		const int kDoubleSolenoidForward = 2;
		const int kDoubleSolenoidReverse = 3;
		Compressor  *compressor;

	void turtleMode()
	{
		MLeft1.Set(m_stick.GetY()*-0.285);
		MLeft2.Set(m_stick.GetY()*-0.285);
		MRight1.Set(m_stick.GetThrottle()*0.285);
		MRight2.Set(m_stick.GetThrottle()*0.285);
	}
	void speedControl()
	{
		if(Sob5 != m_stick.GetRawButton(5))
		{
			Sob5 = !Sob5;
			if(Sob5)
			{
			Sol = Sol+0.1;
			printf("Sol=%f Sor=%f\n",Sol,Sor);
			}
		}
		if(Sob6 != m_stick.GetRawButton(6))
		{
			Sob6 = !Sob6;
			if(Sob6)
			{
				Sor = Sor+0.1;
				printf("Sol=%f Sor=%f\n",Sol,Sor);
			}
		}
		if(Sob7 != m_stick.GetRawButton(7))
		{
			Sob7 = !Sob7;
			if(Sob7)
			{
				Sol = Sol-0.1;
				printf("Sol=%f Sor=%f\n",Sol,Sor);
			}
		}
		if(Sob8 != m_stick.GetRawButton(8))
		{
			Sob8 = !Sob8;
			if(Sob8)
			{
				Sor = Sor-0.1;
				printf("Sol=%f Sor=%f\n",Sol,Sor);
			}
		}
		if(Sob12 != m_stick.GetRawButton(12))
		{
			Sob12 = !Sob12;
			if(Sob12)
			{
				Sol = 0.5;
				Sor = 0.5;
				printf("Sol=%f Sor=%f\n",Sol,Sor);
			}
		}
	}
	void goForward()
	{
		MLeft1.Set(Sol);
		MLeft2.Set(Sol);
		MRight1.Set(-Sor);
		MRight2.Set(-Sor);
	}
	void goForwardMMR()
	{
		MLeft1.Set(Sol+0.2);
		MLeft2.Set(Sol+0.2);
		MRight1.Set(-Sor);
		MRight2.Set(-Sor);
	}
	void goForwardMML()
	{
		MLeft1.Set(Sol);
		MLeft2.Set(Sol);
		MRight1.Set(-Sor-0.2);
		MRight2.Set(-Sor-0.2);
	}
	void goBackward()
	{
		MLeft1.Set(-Sol);
		MLeft2.Set(-Sol);
		MRight1.Set(Sor);
		MRight2.Set(Sor);
	}
	void goBackwardMMR()
	{
		MLeft1.Set(-Sol+0.2);
		MLeft2.Set(-Sol+0.2);
		MRight1.Set(Sor);
		MRight2.Set(Sor);
	}
	void goBackwardMML()
	{
		MLeft1.Set(-Sol);
		MLeft2.Set(-Sol);
		MRight1.Set(Sor-0.2);
		MRight2.Set(Sor-0.2);
	}
	void turnRight()
	{
		MLeft1.Set(Sol);
		MLeft2.Set(Sol);
		MRight1.Set(Sor);
		MRight2.Set(Sor);
	}
	void turnLeft()
	{
		MLeft1.Set(-Sol);
		MLeft2.Set(-Sol);
		MRight1.Set(-Sor);
		MRight2.Set(-Sor);
	}


	void allStop()
	{
		MLeft1.Set(0);
		MLeft2.Set(0);
		MRight1.Set(0);
		MRight2.Set(0);
	}
	void timerprocessor()
	{
		gettimeofday(&tm, NULL);
		long delta = (tm.tv_sec - tm_last.tv_sec) * 1000
					+(tm.tv_usec - tm_last.tv_usec) / 1000;
		if(delta > 10)
		{
			tm_last.tv_sec = tm.tv_sec;
			tm_last.tv_usec = tm.tv_usec;
			if(timer1 > 0)
			{
				timer1--;
			}
		}
	}
public:
	Robot() :
		    m_stick(0),
			m_stick1(1),
			m_stick2(2),// Initialize Joystick on port 0.
			MLeft1(0),
			MLeft2(1),
			MRight1(2),
			MRight2(3),// Initialize the Talon on channel 0.
			ShooterL(4),
			ShooterR(5),
			gyro(0),
			m_solenoid(0), // Use solenoid on channel 0.
			// Use double solenoid with Forward Channel of 1 and Reverse of 2.
			m_doubleSolenoid(1, 2)
	{
	}
	void RobotInit()
		{
			compressor=new Compressor(0);
			compressor->SetClosedLoopControl(true);
			Sol = 0.5;
			Sor = 0.5;
			autoDriveMode = false;
			counter = 0;
			Sob5 = false;
			Sob6 = false;
			Sob7 = false;
			Sob8 = false;
			sob11 = false;
			Sob12 = false;
			timer1 = -1;
			gyro.Reset();
			gettimeofday(&tm_last, NULL);
			printf("ManualMode\n");
			printf("Sol=%f Sor=%f\n",Sol,Sor);
		}
    void Autonomous()
    {
    	int t;
    	t = 1;
    	printf("auto\n");
    	if (IsAutonomous() && IsEnabled())
    	{
    		MLeft1.Set(0.1);
    		MLeft2.Set(0.1);
    		MRight1.Set(0.1);
    		MRight2.Set(0.1);
    		Wait(10);
    		MLeft1.Set(0.3);
    	    MLeft2.Set(0.3);
    		MRight1.Set(-0.3);
    		MRight2.Set(-0.3);
    		Wait(0.5);
    		MLeft1.Set(0);
    		MLeft2.Set(0);
    		MRight1.Set(0);
    		MRight2.Set(0);
    		printf("done\n");
    		m_solenoid.Set(true);
    		if (t > 0)
    		{
    			m_doubleSolenoid.Set(DoubleSolenoid::kForward);
    			printf("done\n");
    	}
    }
}
	void OperatorControl() {
		printf("teleop\n");
		while (IsOperatorControl() && IsEnabled()) {
			timerprocessor();
			if(timer1 == 0)
			{
				printf("timer1 timeout\n");
				timer1 = -1;
			}
			if(sob11 != m_stick.GetRawButton(11)) {
				sob11 = !sob11;
				if(sob11){
					autoDriveMode = !autoDriveMode;
					if(autoDriveMode)
						printf("AutoModeON\n");
					else
						printf("ManualModeON\n");
				}
			}

			if(autoDriveMode)
			{
				speedControl();
				if(m_stick.GetRawButton(9))
				{
					gyro.Reset();
				}
				if(m_stick.GetRawButton(4))
				{
					float angle = gyro.GetAngle();
					if(angle >= 0.3)
					{
						goForwardMML();
					}
					else if(angle <= -0.3)
					{
						goForwardMMR();
					}
					else
					{
						goForward();
					}
				}
				else if(m_stick.GetRawButton(2))
				{
					float angle = gyro.GetAngle();
					if(angle >= 0.3)
					{
						goBackwardMML();
					}
					else if(angle <= -0.3)
					{
						goBackwardMMR();
					}
					else
					{
						goBackward();
					}
				}
				else if(m_stick.GetRawButton(1))
				{
					turnLeft();
				}
				else if(m_stick.GetRawButton(3))
				{
					turnRight();
				}
				else
				{
					allStop();
				}
				ShooterL.Set(m_stick.GetY());
				ShooterR.Set(m_stick.GetY()*-1);
			}
			else
			{
				if(m_stick.GetRawButton(4))
				{
					turtleMode();
				}
				else
				{
					speedControl();
					MLeft1.Set(m_stick.GetY()*Sol*-1);
					MLeft2.Set(m_stick.GetY()*Sol*-1);
					MRight1.Set(m_stick.GetThrottle()*Sor);
					MRight2.Set(m_stick.GetThrottle()*Sor);
					if(m_stick.GetRawButton(3))
					{
						printf("damn\n");
						printf("%f\n",gyro.GetAngle());
					}
				}
		}
			/*	if (m_stick.GetRawButton(kDoubleSolenoidForward))
				{
					m_doubleSolenoid.Set(DoubleSolenoid::kForward);
				    printf("fuck\n");
				}
				else if (m_stick.GetRawButton(kDoubleSolenoidReverse))
				{
					m_doubleSolenoid.Set(DoubleSolenoid::kReverse);
					printf("shit\n");
				}
				else
				{
					m_doubleSolenoid.Set(DoubleSolenoid::kOff);
					Wait(kUpdatePeriod); // Wait 5ms for the next update.
				}*/
		}
	}
};

START_ROBOT_CLASS(Robot);
