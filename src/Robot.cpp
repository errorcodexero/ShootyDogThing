#include <iostream>
#include <memory>
#include <string>
#include <iostream>
#include <cstdio>

#include <DriverStation.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <Relay.h>
#include <CANTalon.h>
#include <Joystick.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Tables/ITableListener.h>

class Shooter : public frc::LiveWindowSendable, ITableListener
{
private:
    std::string m_name;
    CANTalon m_talon;
    bool m_enabled;
    double m_p, m_i, m_d, m_f, m_izone;
    double m_currentLimit, m_rampRate;
    double m_vbus, m_vout, m_iout;
    double m_setpoint, m_encVel, m_speed;
    int m_err;
    int m_faults, m_sticky;
    std::shared_ptr<ITable> m_table;

public:
    Shooter( const std::string& name, int id ) :
    	m_name(name),
	m_talon(id),
	m_enabled(false),
	m_p(0.), m_i(0.), m_d(0.), m_f(0.), m_izone(0.),
	m_currentLimit(0.), m_rampRate(0.),
	m_vbus(0.), m_vout(0.), m_iout(0.),
	m_setpoint(0.), m_encVel(0.), m_speed(0.), m_err(0),
	m_faults(0), m_sticky(0),
	m_table(nullptr)
    {
    	LiveWindow::GetInstance()->AddActuator(m_name, "Talon", m_talon);
    }

    ~Shooter()
    {
    	m_talon.Disable();
    }

    std::string GetSmartDashboardType() const
    {
	return "PIDController";
    }

    std::string GetName() const
    {
    	return m_name;
    }

    void Init()
    {
	m_talon.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	m_talon.ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
	m_talon.SetControlMode(CANSpeedController::kSpeed);
	m_talon.SetFeedbackDevice(CANTalon::QuadEncoder);
	// m_talon.ConfigEncoderCodesPerRev(400);
	m_talon.SetSensorDirection(false);

	// get initial settings from Talon
	m_enabled = false;
	m_p = m_talon.GetP();
	m_i = m_talon.GetI();
	m_d = m_talon.GetD();
	m_f = m_talon.GetF();
	m_izone = m_talon.GetIzone();
//	m_currentLimit = m_talon.GetCurrentLimit();
//	m_rampRate = m_talon.GetClosedLoopRampRate();
	m_setpoint = 0.;

	SmartDashboard::PutData(m_name, this);
    }

    void InitTable(std::shared_ptr<ITable> subTable)
    {
	if (m_table != NULL) {
	    m_table->RemoveTableListener(this);
	}

	m_table = subTable;

	if (m_table != NULL) {
	    m_table->PutBoolean("enabled", m_enabled);
	    m_table->PutNumber("PID Type", 1); // Rate
	    m_table->PutNumber("p", m_p);
	    m_table->PutNumber("i", m_i);
	    m_table->PutNumber("d", m_d);
	    m_table->PutNumber("f", m_f);
	    m_table->PutNumber("izone", m_izone);
	    m_table->PutNumber("currentLimit", m_currentLimit);
	    m_table->PutNumber("rampRate", m_rampRate);
	    m_table->PutNumber("setpoint", m_setpoint);
	    UpdateTable();
	    m_table->AddTableListener(this, false);
	}
    }

    void UpdateTable()
    {
	m_vbus = m_talon.GetBusVoltage();
	m_vout = m_talon.GetOutputVoltage();
	m_iout = m_talon.GetOutputCurrent();
	m_encVel = m_talon.GetEncVel();
	m_speed = m_talon.GetSpeed();
	m_err = m_talon.GetClosedLoopError();
	m_faults = m_talon.GetFaults();
	m_sticky = m_talon.GetStickyFaults();

	if (m_table != NULL) {
	    m_table->PutBoolean("enabled", m_enabled);
	}

	SmartDashboard::PutNumber(m_name + " vbus", m_vbus);
	SmartDashboard::PutNumber(m_name + " vout", m_vout);
	SmartDashboard::PutNumber(m_name + " iout", m_iout);
	SmartDashboard::PutNumber(m_name + " encVel", m_encVel);
	SmartDashboard::PutNumber(m_name + " speed", m_speed);
	SmartDashboard::PutNumber(m_name + " err", m_err);

	SmartDashboard::PutBoolean(m_name + " tempFault",
	  (m_faults & CANSpeedController::kTemperatureFault) != 0);
	SmartDashboard::PutBoolean(m_name + " tempFaultSticky",
	  (m_sticky & CANSpeedController::kTemperatureFault) != 0);
	SmartDashboard::PutBoolean(m_name + " vbusFault",
	  (m_faults & CANSpeedController::kBusVoltageFault) != 0);
	SmartDashboard::PutBoolean(m_name + " vbusFaultSticky",
	  (m_sticky & CANSpeedController::kBusVoltageFault) != 0);
    }

    void ValueChanged(ITable* source, llvm::StringRef key, std::shared_ptr<nt::Value> value, bool isNew)
    {
	if (isNew) {
	    return;
	}

	std::cout << m_name << "::ValueChanged( " << key << " = ";
	if (value->IsBoolean()) {
	    std::cout << value->GetBoolean();
	} else if (value->IsDouble()) {
	    std::cout << value->GetDouble();
	} else if (value->IsString()) {
	    std::cout << value->GetString();
	} else {
	    std::cout << "?";
	}
	std::cout << " )" << std::endl;

	if (key == "enabled" && value->IsBoolean()) {
	    SetEnabled(value->GetBoolean());
	}
	else if (key == "p" && value->IsDouble()) {
	    SetP(value->GetDouble());
	}
	else if (key == "i" && value->IsDouble()) {
	    SetI(value->GetDouble());
	}
	else if (key == "d" && value->IsDouble()) {
	    SetD(value->GetDouble());
	}
	else if (key == "f" && value->IsDouble()) {
	    SetF(value->GetDouble());
	}
	else if (key == "izone" && value->IsDouble()) {
	    SetIzone(value->GetDouble());
	}
	else if (key == "currentLimit" && value->IsDouble()) {
	    SetCurrentLimit(value->GetDouble());
	}
	else if (key == "rampRate" && value->IsDouble()) {
	    SetRampRate(value->GetDouble());
	}
	else if (key == "setpoint" && value->IsDouble()) {
	    SetSetpoint(value->GetDouble());
	}
    }

    double GetP() { return m_p; }

    void SetP( double p )
    {
	if (p != m_p) {
	    std::cout << "SetP(" << p << ")" << std::endl;
	    m_p = p;
	    m_talon.SetP(p);
	}
    }

    double GetI() { return m_i; }

    void SetI( double i )
    {
    	if (i != m_i) {
	    std::cout << "SetI(" << i << ")" << std::endl;
	    m_i = i;
	    m_talon.SetI(i);
    	}
    }

    double GetD() { return m_d; }

    void SetD( double d )
    {
    	if (d != m_d) {
	    std::cout << "SetD(" << d << ")" << std::endl;
	    m_d = d;
	    m_talon.SetD(d);
    	}
    }

    double GetF() { return m_f; }

    void SetF( double f )
    {
    	if (f != m_f) {
	    std::cout << "SetF(" << f << ")" << std::endl;
	    m_f = f;
	    m_talon.SetF(f);
    	}
    }

    double GetIzone() { return m_izone; }

    void SetIzone( double izone )
    {
	if (izone != m_izone) {
	    std::cout << "SetIzone(" << izone << ")" << std::endl;
	    m_izone = izone;
	    m_talon.SetIzone(izone);
	}
    }

    double GetCurrentLimit() { return m_currentLimit; }

    void SetCurrentLimit( double currentLimit )
    {
	if (currentLimit != m_currentLimit) {
	    std::cout << "SetCurrentLimit(" << currentLimit << ")" << std::endl;
	    m_currentLimit = currentLimit;
	    m_talon.SetCurrentLimit(currentLimit);
	}
    }

    double GetRampRate() { return m_rampRate; }

    void SetRampRate( double rampRate )
    {
	if (rampRate != m_rampRate) {
	    std::cout << "SetRampRate(" << rampRate << ")" << std::endl;
	    m_rampRate = rampRate;
	    // sic: "CloseLoop" should be "ClosedLoop".
	    m_talon.SetCloseLoopRampRate(rampRate);
	}
    }

    double GetSetpoint() { return m_setpoint; }

    void SetSetpoint( double setpoint )
    {
	if (setpoint != m_setpoint) {
	 // std::cout << "SetSetpoint(" << setpoint << ")" << std::endl;
	    m_setpoint = setpoint;
	    m_talon.Set(setpoint);
	}
    }

    double Get() { return GetSetpoint(); }

    void Set( double rpm )
    {
	// convert RPM to quadrature-encoder edges per 0.1 sec
	double native = rpm * 1600. / 600.;
	SmartDashboard::PutNumber(m_name + "/setpoint", native);
	SetSetpoint(native);
    }

    bool IsEnabled() { return m_enabled; }

    void Enable()
    {
	if (!m_enabled) {
	    std::cout << "Enable()" << std::endl;
	    m_talon.ClearStickyFaults();
	    m_talon.Enable();
	    m_talon.Set(m_setpoint);
	    m_enabled = true;
	}
    }

    void Disable()
    {
	if (m_enabled) {
	    std::cout << "Disable()" << std::endl;
	    m_talon.Set(0.);
	    m_talon.Disable();
	    m_enabled = false;
	}
    }

    void SetEnabled( bool enabled )
    {
	if (enabled) {
	    Enable();
	} else {
	    Disable();
	}
    }

    void StartLiveWindowMode()
    {
	if (m_table != nullptr) {
	    m_table->AddTableListener(m_name, this, true);
	}
    }

    void StopLiveWindowMode()
    {
	if (m_table != nullptr) {
	    m_table->RemoveTableListener(this);
	}
    }

    std::shared_ptr<ITable> GetTable() const
    {
    	return m_table;
    }
};

class Robot: public frc::IterativeRobot
{
private:
    frc::LiveWindow* lw;

    frc::Joystick joy;

    Shooter shooter1;
    Shooter shooter2;
    frc::Relay feeder;

public:
    Robot() :
	lw(NULL),
	joy(0),
	shooter1("Shooter1", 1),
	shooter2("Shooter2", 2),
	feeder(0)
    {
	    ;
    }

    void RobotInit() {
	lw = LiveWindow::GetInstance();
	lw->AddActuator("Feeder", "Relay", feeder);
	lw->AddActuator("Robot", "Shooter1", shooter1);
	lw->AddActuator("Robot", "Shooter2", shooter2);

	shooter1.Init();
	shooter2.Init();
    }

    void RobotPeriodic() {
	m_ds.WaitForData();
	// this should be done much less often...
	shooter1.UpdateTable();
    }

    void DisabledInit() {
	shooter1.Disable();
	shooter2.Disable();
	feeder.Set(Relay::kOff);
    }

    void DisabledPeriodic() {
    }

    /*
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * GetString line to get the auto name from the text box below the Gyro.
     *
     * You can add additional auto modes by adding additional comparisons to the
     * if-else structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    void AutonomousInit() override {
	shooter1.Enable();
    }

    void AutonomousPeriodic() {
	// setpoint controlled by SmartDashboard
    }

    void TeleopInit() {
	shooter1.Enable();
    }

    void TeleopPeriodic() {
	static int last = 0;
	// scale axis -1..1 to dial position 0..19
	int dial = (int)((joy.GetRawAxis(0) + 1.) * 9.5 + 0.5);
	if (dial != last) {
	    std::cout << "dial = " << dial << std::endl;
	    last = dial;
	}
	// scale range to 0..600
	shooter1.Set(dial * 600. / 19.);
    }

    void TestPeriodic() {
	lw->Run();
    }
};

START_ROBOT_CLASS(Robot)
