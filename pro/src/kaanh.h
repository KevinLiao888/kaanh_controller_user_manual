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
	
	class moveC_3P : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveC_3P(const std::string &name = "moveC_3P_plan");
		ARIS_REGISTER_TYPE(moveC_3P);
	};


	class moveC_CE : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveC_CE(const std::string &name = "moveC_CE_plan");
		ARIS_REGISTER_TYPE(moveC_CE);
	};

	class moveC_RE : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveC_RE(const std::string &name = "moveC_RE_plan");
		ARIS_REGISTER_TYPE(moveC_RE);
	};





	class moveL_Cos : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveL_Cos(const std::string &name = "moveL_Cos_plan");
		ARIS_REGISTER_TYPE(moveL_Cos);
	};

	class moveL_T : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveL_T(const std::string &name = "moveL_T_plan");
		ARIS_REGISTER_TYPE(moveL_T);
	};



	class moveJ_Cos : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJ_Cos(const std::string &name = "moveJ_Cos_plan");
		ARIS_REGISTER_TYPE(moveJ_Cos);
	};

	class moveJ_T : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJ_T(const std::string &name = "moveJ_T_plan");
		ARIS_REGISTER_TYPE(moveJ_T);
	};


	class moveJM_T : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJM_T(const std::string &name = "moveJM_T_plan");
		ARIS_REGISTER_TYPE(moveJM_T);
	};

	class moveJM_TQ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit moveJM_TQ(const std::string &name = "moveJM_TQ_plan");
		ARIS_REGISTER_TYPE(moveJM_TQ);
	};


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
