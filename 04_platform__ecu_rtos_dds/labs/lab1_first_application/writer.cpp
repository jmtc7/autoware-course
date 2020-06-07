#include "util.hpp"
#include "dds/dds.hpp"

using namespace tutorial;
using namespace org::eclipse;

int main(int argc, char* argv[])
{
	try
	{
		//Create the domain participant
		dds::domain::DomainParticipant dp(cyclonedds::domain::default_id());

		auto topic = dds::topic::Topic<Meter>(dp, "Meter");
		auto pub = dds::pub::Publisher(dp);
		auto dw = dds::pub::DataWriter<Meter>(pub, topic);

		while(true)
		{
			auto meter = readMeter(UtilityKind::WATER);
			dw.write(meter); //'dw << meter' is also valid
			sleep(1);
		}
	}
	catch(...){}

	return 0;
}
