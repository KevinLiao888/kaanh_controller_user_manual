#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>


// \brief 机器人命名空间
// \ingroup aris

namespace kaanh
{
	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
	

	
	class sixRrobot : public aris::plan::Plan
	{
	public:
		
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		

		explicit sixRrobot(const std::string &name = "sixRrobot_plan");
		ARIS_REGISTER_TYPE(sixRrobot);
	};

	

}

#endif