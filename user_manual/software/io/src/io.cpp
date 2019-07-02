#include <iostream>
//程序引用aris库的头文件
#include <aris.hpp>
#include<string>


int main()
{
    std::cout << "start testing IO board" << std::endl;
	
	//创建EtherCAT主站对象
    aris::control::EthercatMaster mst;		
	
	//自动扫描、连接从站
    mst.scan();		
	
	//打印主站扫描的从站个数
    std::cout<<"slave num:"<<mst.slavePool().size()<<std::endl;

	auto&cs = aris::server::ControlServer::instance();
	//1、主站对象mst的成员函数setControlStrategy()创建一个实时线程。其形参即是被调用的实时函数，在实时核中每1ms被调用1次
	//2、被调用函数可以实现主站与从站之间的数据交互，打印服务，log服务。(本例以读、写IO信号，并打印为例)
    mst.setControlStrategy([&]()
    {
        static int count{ 0 };	//count用于计数本函数setControlStrategy()执行的次数
        static std::uint8_t value{ 0x01 };
		
        if (++count % 1000 == 0)	//实时核执行一次周期是1ms，每执行1000次，执行本if条件内的语句
        {
			//控制EtherCAT IO板卡的DO口实现“走马灯”逻辑
            value = value << 1;
            if(value == 0) value = 0x01;
			
			//成员函数mout()是实时核打印函数接口，成员函数lout()是实时核log函数接口
            mst.mout() << "count:" << std::dec << count << std::endl;
			mst.lout() << "count:" << std::dec << count << std::endl;
			
			//1、成员函数ecSlavePool()创建从站vector，在实时核中要使用ecSlavePool()，在非实时核使用SlavePool()，at(1)表示第2个从站，即EtherCAT IO板卡在物理连接层面属于第二个从站
            //2、writePdo是写函数，第一个形参是index，第二个形参是subindex，第三个形参写DO的数值，第四个形参表示写操作的bit数
			mst.slavePool().at(1).writePdo(0x7001,0x02,&value,8);
        }
    });
	
	//启动实时线程
    cs.start();
	
	//非实时线程睡眠100秒钟
    std::this_thread::sleep_for(std::chrono::seconds(100));
	
	//关闭实时线程
    cs.stop();

	return 0;
}
