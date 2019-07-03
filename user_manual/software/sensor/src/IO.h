
#include <memory>
#include <aris.hpp>
#include <atomic>



class Sensor : public aris::plan::Plan
{
public:
	double tempData;
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	auto virtual executeRT(aris::plan::PlanTarget &target)->int;
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

	explicit Sensor(const std::string &name = "Sensor_plan");
	ARIS_REGISTER_TYPE(Sensor);
};



