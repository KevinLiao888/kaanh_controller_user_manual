#ifndef FORCECONTROL_H_
#define FORCECONTROL_H_


#include <memory>
#include <aris.hpp>
#include <atomic>


namespace forcecontrol
{
	//机器人力控参数声明
	constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
    constexpr double f_vel_JRC[6] = { 9,11,6,2.5,0.5,0.319206404 };
    constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
    //constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,2.7,1.419632126,0.319206404 };
	constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
	constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
	constexpr double max_static_vel[6] = { 0.12, 0.12, 0.1, 0.05, 0.05, 0.075 };
    constexpr double f_static_index_JRC[6] = { 1.2, 1.2, 0.8, 1.0, 1.5, 0.8 };
    constexpr double f_static_index[6] = { 0.5, 0.5, 0.5, 0.6, 0.95, 0.8 };


	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;

	class MoveJRC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJRC(const std::string &name = "MoveJRC_plan");
		ARIS_REGISTER_TYPE(MoveJRC);
	};

	class MoveStop : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit MoveStop(const std::string &name = "MoveStop_plan");
		ARIS_REGISTER_TYPE(MoveStop);
	};
}

#endif
