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
	

	class ShowAll : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit ShowAll(const std::string &name = "ShowAll_plan");
		ARIS_REGISTER_TYPE(ShowAll);


	};

	class MoveJR : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJR(const std::string &name = "MoveJR_plan");
		ARIS_REGISTER_TYPE(MoveJR);
	};

	class MovePoint : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~MovePoint();
		explicit MovePoint(const std::string &name = "MovePoint_plan");
		ARIS_REGISTER_TYPE(MovePoint);
		MovePoint(const MovePoint &);
		MovePoint(MovePoint &);
		MovePoint& operator=(const MovePoint &);
		MovePoint& operator=(MovePoint &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class MoveJP : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~MoveJP();
		explicit MoveJP(const std::string &name = "MoveJP_plan");
		ARIS_REGISTER_TYPE(MoveJP);
		MoveJP(const MoveJP &);
		MoveJP(MoveJP &);
		MoveJP& operator=(const MoveJP &);
		MoveJP& operator=(MoveJP &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

}

#endif
