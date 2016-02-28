#include <unistd.h>
#include "WPILib.h"
#include <stdio.h>
#include <sys/time.h>
#include <AnalogGyro.h>
#include <iostream>
#include <math.h>
class Robot : public SampleRobot
{
	public:std::shared_ptr<NetworkTable> table;
	Joystick m_stick;
	Joystick m_stick1;
	AnalogGyro gyro;
	DigitalInput di;
	float Sol;
	float Sor;
	bool Sob12;
	bool Sob2;
	bool Sob4;
	bool Sob5;
	bool Sob6;
	bool Sob7;
	bool Sob8;
	bool sob11;
	bool sos;
	int timer1;
	bool autoDriveMode;
	Talon MLeft1;
	Talon MLeft2;
	Talon MRight1;
	Talon MRight2;
	Talon ShooterL;
	Talon ShooterR;
	Talon AngleModulator;
	timeval tm;
	timeval tm_last;
	Solenoid m_solenoid;
	DoubleSolenoid m_doubleSolenoid;
	const double kUpdatePeriod = 0.005;
	const int kSolenoidButton = 3;
	const int kDoubleSolenoidForward = 1;
	const int kDoubleSolenoidReverse = 2;
	Compressor  *compressor;
	void autoTargeting()
	{
		std::vector<double> areas = table->GetNumberArray("area", llvm::ArrayRef<double>());
		std::vector<double> centerXs = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		std::vector<double> centerYs = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
		int max_i = -1;
		int i;
		double max_area = -1;
		if(areas.size() < 1)
		{
			turnRight();
			printf("areas=%d\n", areas.size());
		}
		else
		{
			for (i = 0; i < areas.size(); i++)
			{
				if (areas[i] > max_area) {
					max_area = areas[i];
					max_i = i;
				}
			}
			float s = fabs(centerYs[i] - 240) * 0.003;
			if(centerXs[i] > 322)
			{
				MLeft1.Set(-0.2);
				MLeft2.Set(-0.2);
				MRight1.Set(-0.2);
				MRight2.Set(-0.2);
				printf("centerXs=%f\n", centerXs[i]);
			} else if(centerXs[i] < 318)
			{
				MLeft1.Set(0.2);
				MLeft2.Set(0.2);
				MRight1.Set(0.2);
				MRight2.Set(0.2);
				printf("centerXs=%f\n", centerXs[i]);
			} else
			{
				MLeft1.Set(0);
				MLeft2.Set(0);
				MRight1.Set(0);
				MRight2.Set(0);
			}
			if(centerYs[i] > 241)
			{
				AngleModulator.Set(-0.2-s);
			}else if(centerYs[i] < 239)
			{
				AngleModulator.Set(0.2+s);
			}else
			{
				AngleModulator.Set(0);
			}
		}
	}
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
		AngleModulator.Set(0);
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
			MLeft1(1),
			MLeft2(2),
			MRight1(3),
			MRight2(4),// Initialize the Talon on channel 0.
			ShooterR(5),
			ShooterL(6),
			AngleModulator(7),
			gyro(0),
			m_solenoid(0), // Use solenoid on channel 0.
			// Use double solenoid with Forward Channel of 1 and Reverse of 2.
			m_doubleSolenoid(1, 2),
			di(0)
	{
	}
	void RobotInit()
		{
			if (fork() == 0)
			{
				system("/usr/local/frc/JRE//bin/java -Xmx50m -XX:-OmitStackTraceInFastThrow -XX:+HeapDumpOnOutOfMemoryError -jar '/home/lvuser/grip.jar' '/home/lvuser/project.grip' > /dev/null");
				return;
			}
			printf("shit\n");
			table = NetworkTable::GetTable("/GRIP/Shit");
			compressor=new Compressor(0);
			compressor->SetClosedLoopControl(true);
			Sol = 0.5;
			Sor = 0.5;
			autoDriveMode = false;
			sos = false;
			Sob2 = false;
			Sob4 = false;
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
			if(m_stick.GetRawButton(2))
			{
				compressor->SetClosedLoopControl(true);
			}
			if(m_stick.GetRawButton(3))
			{
				compressor->SetClosedLoopControl(false);
			}
			if(autoDriveMode)
			{
				speedControl();
				if(Sob4 != m_stick.GetRawButton(4))
				{
					Sob4 = !Sob4;
					if(Sob4)
					{
						gyro.Reset();
					}
				}
				if(Sob2 != m_stick.GetRawButton(2))
				{
					Sob2 = !Sob2;
					if(Sob2)
					{
						gyro.Reset();
					}
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
				else if(m_stick1.GetRawButton(10))
				{
					autoTargeting();
				}
				else
				{
					allStop();
				}
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
					ShooterL.Set(m_stick1.GetZ());
					ShooterR.Set(m_stick1.GetZ()*-1);
					AngleModulator.Set(m_stick1.GetY()*Sol);
					if (m_stick1.GetRawButton(kDoubleSolenoidForward))
					{
						m_doubleSolenoid.Set(DoubleSolenoid::kForward);
					    //printf("fuck\n");
					}
					else if (m_stick1.GetRawButton(kDoubleSolenoidReverse))
					{
						m_doubleSolenoid.Set(DoubleSolenoid::kReverse);
						//printf("shit\n");
					}
					else
					{
						m_doubleSolenoid.Set(DoubleSolenoid::kOff);
						Wait(kUpdatePeriod); // Wait 5ms for the next update.
					}
				}
			}
		}
	}
};
START_ROBOT_CLASS(Robot)
