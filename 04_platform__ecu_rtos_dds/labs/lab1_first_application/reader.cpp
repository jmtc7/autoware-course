// -- Std C++ Include
#include<iostream>
#include<algorithm>

#include <dds/dds.hpp>
#include "Meter_DCPS.hpp"
#include "util.hpp"

using namespace tutorial;
using namespace org::eclipse;

int main(int argc, char* argv[])
{
	try
	{
		dds::domain::DomainParticipant dp(cyclonedds::domain::default_id());

		auto topic = dds::topic::Topic<Meter>(dp, "Meter");
		auto sub = dds::pub::Subscriber(dp);
		auto dr = dds::pub::DataReader<Meter>(sub, topic);

		while(true)
		{
			dds::sub::LoanedSamples<Meter> samples;
			samples = dr.read();

			for(auto it=samples.begin(); it!=samples.end(); ++it)
			{
				std::cout << it->data() << std::endl;
			}

			sleep(1);
		}
	}
	catch(...)
	{
		std::cout << "[!] ERROR! Something went wrong" << std::endl;
	}

	return 0;
}
