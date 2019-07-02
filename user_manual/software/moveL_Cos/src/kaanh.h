#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>

namespace kaanh
{
	//模型参数配置
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
	//声明指令printing
	class moveL_Cos : public aris::plan::Plan
	{
	public:
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		explicit moveL_Cos(const std::string &name = "moveL_Cos_plan");
		ARIS_REGISTER_TYPE(moveL_Cos);
	};
}
#endif
