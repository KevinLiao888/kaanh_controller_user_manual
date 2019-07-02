#include <iostream>
#include <aris.hpp>
#include "kaanh.h"
#include<atomic>
#include<string>
#include<filesystem>


using namespace aris::dynamic;
auto xmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "rokae.xml";
int main(int argc, char *argv[])
{
	std::cout << "new" << std::endl;
	xmlpath = xmlpath / xmlfile;
	std::cout << xmlpath << std::endl;
	auto&cs = aris::server::ControlServer::instance();
	auto port = argc < 2 ? 5866 : std::stoi(argv[1]);

	//生成rokae.xml文档
	/*
	//-------for rokae robot begin//
	cs.resetController(kaanh::createControllerRokaeXB4().release());
	cs.resetModel(kaanh::createModelRokae().release());
	cs.resetPlanRoot(kaanh::createPlanRootRokaeXB4().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.saveXmlFile(xmlpath.string().c_str());
	//-------for rokae robot end//
	*/

	cs.loadXmlFile(xmlpath.string().c_str());
	cs.start();

	//Start Web Socket//
	cs.startWebSock("5866");

	//Receive Command//
	cs.runCmdLine();

	return 0;
}
