/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/Logger.hpp>

using namespace bus_schremote;

const bool Task::AnIn[12]={0,0,0,0,1,1,1,1,1,1,0,0};
const bool Task::I2C[12]={1,2,0,0,0,0,0,0,1,2,0,0};
const bool Task::UART_SPI_CNT[12]={1,1,1,0,1,1,1,1,1,1,1,0};
const unsigned short Task::UARTbuffer = UARTbufferMAX;

Task::Task(std::string const& name)
    : TaskBase(name), srh(NULL), uart_0_index(-1), uart_1_index(-1)
{
	memset(PinUse,0,12);

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), srh(NULL), uart_0_index(-1), uart_1_index(-1)
{
	memset(PinUse,0,12);
}

Task::~Task()
{
}

bool Task::portsConfig(){
	{/* Configuring Digital Orogen-Outputs (Digital Device-Input)  */
	RTT::OutputPort<raw_io::Digital>* new_output_port;
	DsConfig din_vector = _digital_ins.get();
	for(DsConfig::iterator it = din_vector.begin(); it != din_vector.end(); ++it) {

		//Warning pin in use
		if(PinUse[it->pin])
			RTT::log(RTT::Warning) << "Two devices using the same pin. Pin configuration will be overwritten." << RTT::endlog();

		//Setting pin up
		if(!sr_pin_setup(srh,it->pin,sr_pt_din_pullup)){
			RTT::log(RTT::Error) << it->name << " could not be set up at pin " << it->pin << 
						". The followin error was received: " << sr_error_info(srh) << RTT::endlog();
		continue;
		}
		PinUse[it->pin]=true;
		
		// Creating Orogen component ports
		new_output_port = new RTT::OutputPort<raw_io::Digital>(it->name);
		ports()->addPort(it->name, *new_output_port);
		Din din = {*it, new_output_port};
		din_mapping.push_back(din);	

	}}

	{/* Configuring Digital Orogen-Input (Digital Device-Output)  */
	RTT::InputPort<raw_io::Digital>* new_input_port;
	DsConfig dout_vector = _digital_outs.get();
	for(DsConfig::iterator it = dout_vector.begin(); it != dout_vector.end(); ++it) {

		//Warning pin in use
		if(PinUse[it->pin])
			RTT::log(RTT::Warning) << "Two devices using the same pin. Pin configuration will be overwritten." << RTT::endlog();

		//Setting pin up
		if(!sr_pin_setup(srh,it->pin,sr_pt_dout_low)){
			RTT::log(RTT::Error) << it->name << " could not be set up at pin " << it->pin << 
						". The followin error was received: " << sr_error_info(srh) << RTT::endlog();
		continue;
		}
		PinUse[it->pin]=true;
		
		// Creating Orogen component ports
		new_input_port = new RTT::InputPort<raw_io::Digital>(it->name);
		ports()->addPort(it->name, *new_input_port);
		Dout dout = {*it, new_input_port};
		dout_mapping.push_back(dout);	

	}}



	{/* Configuring UART Orogen-Outputs (UART Device-Input)  */
	RTT::OutputPort<iodrivers_base::RawPacket>* new_output_port;
	RTT::InputPort<iodrivers_base::RawPacket>* new_input_port;
	UARTsConfig uart_vector = _uarts.get();
	bool inverted_logic = false;
	for(UARTsConfig::iterator it = uart_vector.begin(); it != uart_vector.end(); ++it) {
		//Checking UART capable pin
		if(!(UART_SPI_CNT[it->rx]&&UART_SPI_CNT[it->tx])){
			RTT::log(RTT::Error) << "Non UART pin selected for: " + it->name + ". Ports could not be created" << RTT::endlog();
			continue;
		}
		
		//Warning pin in use
		if(PinUse[it->tx]||PinUse[it->rx])
			RTT::log(RTT::Warning) << "Two devices using the same pin. Pin configuration will be overwritten." << RTT::endlog();
		
		
		
		
		//Pin configuration with error checking

		//rx
		if(!sr_pin_setup(srh,it->rx,sr_pt_din_pullup)){
			RTT::log(RTT::Error) << "rx UART could not be set up at pin " << it->rx << 
						". The followin error was received: " << sr_error_info(srh) << RTT::endlog();
			continue;
		}

		//tx
		bool tx_pin_done;
		if(inverted_logic)
			tx_pin_done = sr_pin_setup(srh,it->tx,sr_pt_dout_opendrain_short);
		else
			tx_pin_done = sr_pin_setup(srh,it->tx,sr_pt_dout_opendrain_open);

		if(!tx_pin_done){
			RTT::log(RTT::Error) << "tx UART could not be set up at pin " << it->tx << 
						". The following error was received: " << sr_error_info(srh) << RTT::endlog();
			continue;
		}


		//UART mode check
		if(it->mode > 5){
			RTT::log(RTT::Error) << "Mode " << it->mode << " do not exist. Try using a combination (by OR operation) of parity mode and stopbit mode from MODES enum." << RTT::endlog();
			continue;
		}

		
		//UART module check
		if(it->uart_module == 0)
			it->uart_module = 0;
		else{
			if(it->uart_module != 1)
				RTT::log(RTT::Warning) << "UART module "<< it->uart_module << " do not exist. Loading at UART module 1." << RTT::endlog();
			it->uart_module = 1;				
		}
		
		PinUse[it->rx]=true;
		PinUse[it->tx]=true;

		// Creating Orogen component ports
		new_output_port = new RTT::OutputPort<iodrivers_base::RawPacket>(it->name+"_out");
		new_input_port = new RTT::InputPort<iodrivers_base::RawPacket>(it->name+"_in");
		ports()->addPort(it->name+"_out", *new_output_port);
		ports()->addPort(it->name+"_in", *new_input_port);
		UART uart = {*it, new_output_port, new_input_port};
		if(it->uart_module==1)			
			uart_1_mapping.push_back(uart);
		else
			uart_0_mapping.push_back(uart);
			
	}}

	//Setting UART up
	//module 0
	if(uart_0_mapping.size()>0)
		if(!sr_uart_enable(srh, uart_0_mapping.front().uartconfig.uart_module, 
					uart_0_mapping.front().uartconfig.mode, 
					uart_0_mapping.front().uartconfig.baud, 
					uart_0_mapping.front().uartconfig.rx, 
					uart_0_mapping.front().uartconfig.tx))
			RTT::log(RTT::Error) << uart_0_mapping.front().uartconfig.name << 
					" could not be set up at rx pin " << uart_0_mapping.front().uartconfig.rx << 
					" and tx pin " << uart_0_mapping.front().uartconfig.tx << 
					". The followin error was received: " << sr_error_info(srh) << RTT::endlog();
		else
			uart_0_index = 0;
		
	
	//module 1
	if(uart_1_mapping.size()>0)
		if(!sr_uart_enable(srh, uart_1_mapping.front().uartconfig.uart_module, 
					uart_1_mapping.front().uartconfig.mode, 
					uart_1_mapping.front().uartconfig.baud, 
					uart_1_mapping.front().uartconfig.rx, 
					uart_1_mapping.front().uartconfig.tx))
			RTT::log(RTT::Error) << uart_1_mapping.front().uartconfig.name << 
					" could not be set up at rx pin " << uart_1_mapping.front().uartconfig.rx << 
					" and tx pin " << uart_1_mapping.front().uartconfig.tx << 
					". The followin error was received: " << sr_error_info(srh) << RTT::endlog();
		else		
			uart_1_index = 0;
		


	/*
		if(type == sr_pt_analog_in && AnIn[it->pin])
			throw std::domain_error("Pin not available for analog input.");*/
	return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
	
	const char* bcast_addr = _ip.get().c_str();
	
	if(_mac.get()==""){
		memcpy(ip_addr, bcast_addr,strlen(bcast_addr)+1);
		srh = sr_open_eth(ip_addr);
		return (srh!=NULL && portsConfig());
	}

	unsigned char mac[6];

	sscanf(_mac.get().c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	

	SR_IPLIST *list = sr_discover(bcast_addr);
	if (list == NULL)
	{  
		//std::cerr << "Discover: FAIL\n could not find any device." << std::endl;
		RTT::log(RTT::Error) << "Discover: FAIL. could not find any device." << RTT::endlog();
		return false;
	}
	else
	{
		SR_IPLIST *l = list;
		while (l != NULL)
		{
			if(memcmp ( mac, l->mac, 6 ))
				l = l->next;
			else
			{
				sprintf(ip_addr,"%u.%u.%u.%u",  (unsigned int) l->ip & 0xff, 
								(unsigned int) (l->ip>>8) & 0xff, 
								(unsigned int) (l->ip>>16) & 0xff, 
								(unsigned int) (l->ip>>24) & 0xff);
				RTT::log(RTT::Info) << "Device IP: " << ip_addr << RTT::endlog();				
				srh = sr_open_eth(ip_addr);
				sr_discover_free(list);
				return (srh!=NULL && portsConfig());
			}

		}
		//IF NOT FOUND - show all MAC/IP found
		RTT::log(RTT::Error) << _mac.get() << " device not found." << RTT::endlog();
		RTT::log(RTT::Warning) << "Devices found: \n";
		l = list;
		while (l != NULL)
		{
			sprintf(ip_addr,"%u.%u.%u.%u",  (unsigned int) l->ip & 0xff, 
							(unsigned int) (l->ip>>8) & 0xff, 
							(unsigned int) (l->ip>>16) & 0xff, 
							(unsigned int) (l->ip>>24) & 0xff);
			RTT::log(RTT::Warning) <<  "ip:" << ip_addr ;

      			sprintf(mac_addr,"%02X-%02X-%02X-%02X-%02X-%02X",(int)l->mac[0], 
									(int)l->mac[1], 
									(int)l->mac[2], 
									(int)l->mac[3], 
									(int)l->mac[4], 
									(int)l->mac[5]);
			RTT::log(RTT::Warning) <<  "\tmac:" << mac_addr << " \n";
			l = l->next;	
		}
			RTT::log(RTT::Warning) <<  RTT::endlog();
		
		sr_discover_free(list);
    		return false;
	}

}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
	
	//Digital output
	raw_io::Digital out;
	for(std::vector<Din>::iterator it = din_mapping.begin(); it != din_mapping.end(); ++it) {
		if(!sr_pin_get(srh,it->pinconfig.pin,&(out.data))){
			RTT::log(RTT::Warning) << "Pin "<< it->pinconfig.pin << " from port " << it->pinconfig.name << " could not be read." << RTT::endlog();
		continue;		
		}
		out.time = base::Time::now();
		 (it->output)->write(out);
	}
	
	//Digital input
	raw_io::Digital in;
	for(std::vector<Dout>::iterator it = dout_mapping.begin(); it != dout_mapping.end(); ++it) {
		(it->input)->read(in);
		if(!sr_pin_set(srh,it->pinconfig.pin,in.data)){
			RTT::log(RTT::Warning) << "Pin "<< it->pinconfig.pin << " from port " << it->pinconfig.name << " could not be written." << RTT::endlog();
		continue;		
		}
	}

	//UART module 0
	if(uart_0_index >= 0){

		//Read UART send through Output port
		iodrivers_base::RawPacket out;		
		if(!sr_uart_read_arr(srh, uart_0_mapping[uart_0_index].uartconfig.uart_module, UARTpacket, UARTbuffer, &UARTcnt))
			RTT::log(RTT::Warning) << "UART from port " << uart_0_mapping[uart_0_index].uartconfig.name << 
						" could not be read. The followin error was received: " << sr_error_info(srh) << 
						RTT::endlog();
		else if(UARTcnt>0){
		out.time = base::Time::now();
		out.data.assign(UARTpacket, UARTpacket + UARTcnt);
		 (uart_0_mapping[uart_0_index].output)->write(out);
		}
	

		//Read Input port send through UART
		iodrivers_base::RawPacket in;
		while (uart_0_mapping[uart_0_index].input->read(in) == RTT::NewData){

			if(!sr_uart_write_arr(srh, uart_0_mapping[uart_0_index].uartconfig.uart_module, &(in.data[0]),in.data.size()))
				RTT::log(RTT::Warning) << "UART from port " << uart_0_mapping[uart_0_index].uartconfig.name << 
							" could not be written. The following error was received: " << sr_error_info(srh) << 
							RTT::endlog();}

		//increment index
		if(uart_0_mapping.size() > 1){
			if(++uart_0_index == uart_0_mapping.size())
				uart_0_index=0;
			
			if(!sr_uart_enable(srh, uart_0_mapping[uart_0_index].uartconfig.uart_module, 
						uart_0_mapping[uart_0_index].uartconfig.mode, 
						uart_0_mapping[uart_0_index].uartconfig.baud, 
						uart_0_mapping[uart_0_index].uartconfig.rx, 
						uart_0_mapping[uart_0_index].uartconfig.tx)){
				RTT::log(RTT::Error) << uart_0_mapping[uart_0_index].uartconfig.name << 
						" could not be set up at rx pin " << uart_0_mapping[uart_0_index].uartconfig.rx << 
						" and tx pin " << uart_0_mapping[uart_0_index].uartconfig.tx << 
						". Stopping module. The following error was received: " << sr_error_info(srh) << RTT::endlog();

				sr_uart_disable(srh,uart_0_mapping[uart_0_index].uartconfig.uart_module);
				uart_0_index=-1;
				}
		}
	}

	//UART module 1
	if(uart_1_index >= 0){
		//Read UART send through Output port
		iodrivers_base::RawPacket out;		
		if(!sr_uart_read_arr(srh, uart_1_mapping[uart_1_index].uartconfig.uart_module, UARTpacket, UARTbuffer, &UARTcnt))
			RTT::log(RTT::Warning) << "UART from port " << uart_1_mapping[uart_1_index].uartconfig.name << 
						" could not be read. The following error was received: " << sr_error_info(srh) << 
						RTT::endlog();
		else if(UARTcnt>0){
		out.time = base::Time::now();
		out.data.assign(UARTpacket, UARTpacket + UARTcnt);
		 (uart_1_mapping[uart_1_index].output)->write(out);
		}
	

		//Read Input port send through UART
		iodrivers_base::RawPacket in;
		while (uart_0_mapping[uart_0_index].input->read(in) == RTT::NewData)
			if(!sr_uart_write_arr(srh, uart_1_mapping[uart_1_index].uartconfig.uart_module, &(in.data[0]),in.data.size()))
				RTT::log(RTT::Warning) << "UART from port " << uart_1_mapping[uart_1_index].uartconfig.name << 
							" could not be written. The following error was received: " << sr_error_info(srh) << 
							RTT::endlog();
		//increment index
		if(uart_1_mapping.size() > 1){
			if(++uart_1_index == uart_1_mapping.size())
				uart_1_index=0;
			
			if(!sr_uart_enable(srh, uart_1_mapping[uart_1_index].uartconfig.uart_module, 
						uart_1_mapping[uart_1_index].uartconfig.mode, 
						uart_1_mapping[uart_1_index].uartconfig.baud, 
						uart_1_mapping[uart_1_index].uartconfig.rx, 
						uart_1_mapping[uart_1_index].uartconfig.tx)){
				RTT::log(RTT::Error) << uart_1_mapping[uart_1_index].uartconfig.name << 
						" could not be set up at rx pin " << uart_1_mapping[uart_1_index].uartconfig.rx << 
						" and tx pin " << uart_1_mapping[uart_1_index].uartconfig.tx << 
						". Stopping module. The following error was received: " << sr_error_info(srh) << RTT::endlog();
				
				sr_uart_disable(srh,uart_1_mapping[uart_1_index].uartconfig.uart_module);
				uart_1_index=-1;
			}
		}
	}
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
	if (!srh) return;
	
	for(std::vector<Din>::iterator it = din_mapping.begin(); it != din_mapping.end(); ++it) {
		ports()->removePort(it->output->getName());
		PinUse[(it->pinconfig).pin]=false;
		delete it->output;
	}
	
	
	for(std::vector<Dout>::iterator it = dout_mapping.begin(); it != dout_mapping.end(); ++it) {
		ports()->removePort(it->input->getName());
		PinUse[(it->pinconfig).pin]=false;
		delete it->input;
	}


	for(std::vector<UART>::iterator it = uart_0_mapping.begin(); it != uart_0_mapping.end(); ++it) {
		ports()->removePort(it->output->getName());
		ports()->removePort(it->input->getName());
		PinUse[(it->uartconfig).rx]=false;
		PinUse[(it->uartconfig).tx]=false;
		delete it->output;
		delete it->input;
	}
	
	for(std::vector<UART>::iterator it = uart_1_mapping.begin(); it != uart_1_mapping.end(); ++it) {
		ports()->removePort(it->output->getName());
		ports()->removePort(it->input->getName());
		PinUse[(it->uartconfig).rx]=false;
		PinUse[(it->uartconfig).tx]=false;
		delete it->output;
		delete it->input;
	}
	
	
	if(uart_0_index>=0)
		sr_uart_disable(srh, 0);
	
	if(uart_1_index>=0)
		sr_uart_disable(srh, 1);
	
	sr_close(srh);
	srh=NULL;
}
