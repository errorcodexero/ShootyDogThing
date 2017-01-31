#include <iostream>
#include <memory>
#include <string>

#include <DriverStation.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <Relay.h>
#include <CANTalon.h>
#include <Joystick.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Tables/ITableListener.h>

class Shooter : public frc::NamedSendable, public frc::LiveWindowSendable, ITableListener
{
private:
    std::string m_name;
    CANTalon m_talon;
    bool m_enabled;
    double m_p, m_i, m_d, m_f, m_izone;
    double m_currentLimit, m_rampRate;
    double m_vbus, m_vout, m_iout;
    double m_speed, m_setpoint;
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
	m_speed(0.), m_setpoint(0.), m_err(0),
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
    	return "Speed Controller";
    }

    std::string GetName() const
    {
    	return m_name;
    }

    void Init()
    {
	m_talon.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	m_talon.ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
	m_talon.SetFeedbackDevice(CANTalon::QuadEncoder);
	m_talon.ConfigEncoderCodesPerRev(400);
	m_talon.SetSensorDirection(false);
	m_talon.SetControlMode(CANSpeedController::kSpeed);
	GetTalon();

	InitTable(NetworkTable::GetTable(m_name));
	UpdateTable();
    }

    double GetP() { return m_p; }

    void SetP( double p )
    {
	if (p != m_p) {
	    m_p = p;
	    m_talon.SetP(p);
	}
	UpdateTable();
    }

    double GetI() { return m_i; }

    void SetI( double i )
    {
    	if (i != m_i) {
	    m_i = i;
	    m_talon.SetI(i);
    	}
    	UpdateTable();
    }

    double GetD() { return m_d; }

    void SetD( double d )
    {
    	if (d != m_d) {
	    m_d = d;
	    m_talon.SetD(d);
    	}
    	UpdateTable();
    }

    double GetF() { return m_f; }

    void SetF( double f )
    {
    	if (f != m_f) {
	    m_f = f;
	    m_talon.SetF(f);
    	}
    	UpdateTable();
    }

    double GetIzone() { return m_izone; }

    void SetIzone( double izone )
    {
	if (izone != m_izone) {
	    m_izone = izone;
	    m_talon.SetIzone(izone);
	}
	UpdateTable();
    }

    double GetCurrentLimit() { return m_currentLimit; }

    void SetCurrentLimit( double currentLimit )
    {
	if (currentLimit != m_currentLimit) {
	    m_currentLimit = currentLimit;
	    m_talon.SetCurrentLimit(currentLimit);
	}
	UpdateTable();
    }

    double GetRampRate() { return m_rampRate; }

    void SetRampRate( double rampRate )
    {
	if (rampRate != m_rampRate) {
	    m_rampRate = rampRate;
	    // sic: "CloseLoop" should be "ClosedLoop".
	    m_talon.SetCloseLoopRampRate(rampRate);
	}
	UpdateTable();
    }

    double GetSetpoint() { return m_setpoint; }

    void SetSetpoint( double setpoint )
    {
	if (setpoint != m_setpoint) {
	    m_setpoint = setpoint;
	    m_talon.Set(setpoint);
	}
	UpdateTable();
    }

    double Get() { return GetSetpoint(); }

    void Set( double setpoint ) { SetSetpoint(setpoint); }

    bool IsEnabled() { return m_enabled; }

    void Enable()
    {
	if (!m_enabled) {
	    m_talon.ClearStickyFaults();
	    m_talon.Set(m_setpoint);
	    m_talon.Enable();
	    m_enabled = true;
	}
    }

    void Disable()
    {
	if (m_enabled) {
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

    void ValueChanged(ITable* source, llvm::StringRef key, std::shared_ptr<nt::Value> value, bool isNew)
    {
	if (isNew) {
	    return;
	}
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
	    Set(value->GetDouble());
	}
    }

    void GetTalon()
    {
	m_enabled = m_talon.IsEnabled();
	m_p = m_talon.GetP();
	m_i = m_talon.GetI();
	m_d = m_talon.GetD();
	m_f = m_talon.GetF();
	m_izone = m_talon.GetIzone();
//	m_currentLimit = m_talon.GetCurrentLimit();
//	m_rampRate = m_talon.GetClosedLoopRampRate();
	m_setpoint = m_talon.GetSetpoint();
	m_vbus = m_talon.GetBusVoltage();
	m_vout = m_talon.GetOutputVoltage();
	m_iout = m_talon.GetOutputCurrent();
	m_speed = m_talon.GetSpeed();
	m_err = m_talon.GetClosedLoopError();
	m_faults = m_talon.GetFaults();
	m_sticky = m_talon.GetStickyFaults();
    }

    void UpdateTable()
    {
	if (m_table != NULL) {
	    m_table->PutBoolean("enabled", m_enabled);

	    std::shared_ptr<ITable> pidTable = m_table->GetSubTable("PID");
	    pidTable->PutNumber("p", m_p);
	    pidTable->PutNumber("i", m_i);
	    pidTable->PutNumber("d", m_d);
	    pidTable->PutNumber("f", m_f);
	    pidTable->PutNumber("izone", m_izone);
	    pidTable->PutNumber("currentLimit", m_currentLimit);
	    pidTable->PutNumber("rampRate", m_rampRate);
	    pidTable->PutNumber("setpoint", m_setpoint);

	    /////

	    std::shared_ptr<ITable> outputTable = m_table->GetSubTable("OUTPUT");
	    outputTable->PutNumber("vbus", m_vbus);
	    outputTable->PutNumber("vout", m_vout);
	    outputTable->PutNumber("iout", m_iout);
	    outputTable->PutNumber("speed", m_speed);
	    outputTable->PutNumber("err", m_err);

	    /////

	    std::shared_ptr<ITable> faultsTable = m_table->GetSubTable("FAULTS");
	    faultsTable->PutBoolean("Temp",
	      (m_faults & CANSpeedController::kTemperatureFault) != 0);
	    faultsTable->PutBoolean("Temp-sticky",
	      (m_sticky & CANSpeedController::kTemperatureFault) != 0);
	    faultsTable->PutBoolean("Vbus",
	      (m_faults & CANSpeedController::kBusVoltageFault) != 0);
	    faultsTable->PutBoolean("Vbus-sticky",
	      (m_sticky & CANSpeedController::kBusVoltageFault) != 0);
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

    void InitTable(std::shared_ptr<ITable> subTable)
    {
	if (m_table != NULL)
	    m_table->RemoveTableListener(this);
	m_table = subTable;
	if (m_table != NULL) {
	    UpdateTable();
	    m_table->AddTableListener(this, false);
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
	shooter1.GetTalon();
	shooter1.UpdateTable();
    }

    void DisabledInit() {
	shooter1.Disable();
	shooter2.Disable();
	feeder.Set(Relay::kOff);
    }

    void DisabledPeriodic() {
	shooter1.Set(0.);
	shooter2.Set(0.);
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
//	    autoSelected = chooser.GetSelected();
//	    std::cout << "Auto selected: " << autoSelected << std::endl;
//
//	    if (autoSelected == autoNameCustom) {
//		// Custom Auto goes here
//	    } else {
//		// Default Auto goes here
//	    }
    }

    void AutonomousPeriodic() {
//	if (autoSelected == autoNameCustom) {
//	    // Custom Auto goes here
//	} else {
//	    // Default Auto goes here
//	}
    }

    void TeleopInit() {
	shooter1.Enable();
    }

    void TeleopPeriodic() {
	// Set range: 0 to 1000RPM
	shooter1.Set((joy.GetRawAxis(0) + 1.) * 500.);
    }

    void TestPeriodic() {
	lw->Run();
    }
};

START_ROBOT_CLASS(Robot)
