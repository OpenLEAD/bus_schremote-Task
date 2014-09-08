/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/Logger.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <iomanip>

using namespace bus_schremote;
using namespace std;
using namespace RTT;
using boost::lexical_cast;

const bool Task::AnIn[Task::NUMBER_OF_PINS]=
    {false,false,false,false,true ,true ,true ,true ,true ,true ,false,false};
const bool Task::I2C[Task::NUMBER_OF_PINS] =
    {true ,true ,false,false,false,false,false,false,true ,true ,false,false};
const bool Task::UART_SPI_CNT[Task::NUMBER_OF_PINS] =
    {true ,true ,true ,false,true ,true ,true ,true ,true ,true ,true ,false};

static string macToString(unsigned char const* mac)
{
    ostringstream formatter;
    formatter << hex << setfill('0') 
        << setw(2) << static_cast<int>(mac[0]) << ":"
        << setw(2) << static_cast<int>(mac[1]) << ":"
        << setw(2) << static_cast<int>(mac[2]) << ":"
        << setw(2) << static_cast<int>(mac[3]) << ":"
        << setw(2) << static_cast<int>(mac[4]) << ":"
        << setw(2) << static_cast<int>(mac[5]);
    return formatter.str();
}

static string ipv4ToString(uint32_t ip)
{
    ostringstream formatter;
    formatter
        << (ip & 0xff) << "."
        << ((ip>>8) & 0xff) << "."
        << ((ip>>16) & 0xff) << "."
        << ((ip>>24) & 0xff);
    return formatter.str();
}

Task::Task(string const& name)
    : TaskBase(name), srh(NULL)
{
}

Task::Task(string const& name, ExecutionEngine* engine)
    : TaskBase(name, engine), srh(NULL)
{
}

Task::~Task()
{
}

void Task::validateDigitalIOConfiguration(
        set<string>& portNames,
        vector<bool>& usedPins,
        DsConfig const& config) const
{
    for(DsConfig::const_iterator it = config.begin(); it != config.end(); ++it){
        if(it->pin < 0)
            throw logic_error("negative pin number found " + lexical_cast<string>(it->pin));
        if(it->pin >= usedPins.size())
            throw logic_error("pin number " + lexical_cast<string>(it->pin) + " too high, the last pin is " + lexical_cast<string>(usedPins.size()));
        if(usedPins[it->pin])
            throw logic_error("digital pin " + lexical_cast<string>(it->pin) + " is used for multiple digital I/O");
        if(portNames.count(it->name))
            throw logic_error("digital pin " + lexical_cast<string>(it->pin) + " configured to use the name '" + it->name + "' which is used by another I/O");

        portNames.insert(it->name);
        usedPins[it->pin] = true;
    }
}

void Task::validateUARTPin(vector<bool> const& usedPins, unsigned int pin, unsigned int module)
{
    if(pin < 0){
        throw logic_error("negative pin number found " +
                lexical_cast<string>(pin) +
                " for UART " + lexical_cast<string>(module));
    }
    if(pin >= usedPins.size()){
        throw logic_error("pin number " + lexical_cast<string>(pin) +
                " too high for UART " + lexical_cast<string>(module) +
                ", the last pin is " + lexical_cast<string>(usedPins.size()));
    }
    if(!UART_SPI_CNT[pin]){
        throw logic_error("pin " + lexical_cast<string>(pin) +
                " configured to be used for UART " + lexical_cast<string>(module) +
                " is not UART-capable");
    }
    if(usedPins[pin]){
        throw logic_error("pin " + lexical_cast<string>(pin) +
                " configured to be used for UART " + lexical_cast<string>(module) +
                " is already in use");
    }
}

void Task::validateUARTConfiguration(
        set<string>& portNames,
        vector<bool>& usedPins,
        UARTsConfig const& uarts)
{
    vector<bool> usedUARTs(UART_MODULES_COUNT, false);

    for(UARTsConfig::const_iterator it = uarts.begin(); it != uarts.end(); ++it) {
        if(it->uart_module < 0 || it->uart_module >= usedUARTs.size())
            throw logic_error("invalid UART module " + lexical_cast<string>(it->uart_module) +
                    " found in configuration. There are " + lexical_cast<string>(usedUARTs.size()) + " UARTS in total");
        if(usedUARTs[it->uart_module]){
            throw logic_error("UART module " + lexical_cast<string>(it->uart_module) +
                    " is configured multiple times");
        }
        usedUARTs[it->uart_module] = true;
        validateUARTPin(usedPins, it->rx, it->uart_module);
        usedPins[it->rx] = true;
        validateUARTPin(usedPins, it->tx, it->uart_module);
        usedPins[it->tx] = true;

        if(it->mode > 5){
            throw logic_error("UART mode " + lexical_cast<string>(it->mode) +
                    " of UART module " + lexical_cast<string>(it->uart_module) +
                    " is invalid");
        }


        if (portNames.count(it->name) > 0)
            throw logic_error("UART " + lexical_cast<string>(it->uart_module) + " is configured to use the name '" + it->name + "' which is used by another I/O");
        if (portNames.count("w" + it->name) > 0)
            throw logic_error("UART " + lexical_cast<string>(it->uart_module) + " is configured to use the name 'w" + it->name + "' which is used by another I/O");

        portNames.insert(it->name);
        portNames.insert("w" + it->name);
    }
}

void Task::validateConfiguration()
{
    set<string> portNames;
    vector<bool> usedPins(NUMBER_OF_PINS, false);

    validateDigitalIOConfiguration(portNames, usedPins, _digital_ins.get());
    validateDigitalIOConfiguration(portNames, usedPins, _digital_outs.get());

    validateUARTConfiguration(portNames, usedPins, _uarts.get());
}

void Task::resetHardware()
{
    for(int i = 0; i < UART_MODULES_COUNT; ++i){
        uarts[i].enabled = false;
        sr_uart_disable(srh, i);
    }
}

void Task::setupHardware()
{
    DsConfig digitals[2] = { _digital_ins.get(), _digital_outs.get() };
    for(int i = 0; i < 2; ++i){
        for(DsConfig::iterator it = digitals[i].begin(); it != digitals[i].end(); ++it){
            if(!sr_pin_setup(srh,it->pin,sr_pt_din_pullup))
                throw runtime_error("cannot setup pin " + lexical_cast<string>(it->pin));
        }
    }

    UARTsConfig uarts = _uarts.get();
    for(UARTsConfig::iterator it = uarts.begin(); it != uarts.end(); ++it) {
        //rx
        if(!sr_pin_setup(srh,it->rx,sr_pt_din_pullup)){
            throw logic_error("RX pin " + lexical_cast<string>(it->rx) +
                    " configured to be used for UART " + lexical_cast<string>(it->uart_module) +
                    " could not be set up: " + string(sr_error_info(srh)));
        }

        //tx
        bool tx_pin_done;
        bool inverted_logic = false;
        if(inverted_logic)
            tx_pin_done = sr_pin_setup(srh,it->tx,sr_pt_dout_opendrain_short);
        else
            tx_pin_done = sr_pin_setup(srh,it->tx,sr_pt_dout_opendrain_open);

        if(!tx_pin_done){
            throw logic_error("TX pin " + lexical_cast<string>(it->tx) +
                    " configured to be used for UART " + lexical_cast<string>(it->uart_module) +
                    " could not be set up: " + string(sr_error_info(srh)));
        }

        if(!sr_uart_enable(srh, it->uart_module, it->mode, it->baud, it->rx, it->tx)){
            throw logic_error("could not enable UART module " + lexical_cast<string>(it->uart_module));
        }
    }
}

    template<typename MappingType>
void Task::createDigitalPorts(DsConfig const& config, MappingType& mapping)
{
    for(DsConfig::const_iterator it = config.begin(); it != config.end(); ++it) {
        // Creating Orogen component ports
        typedef typename MappingType::value_type::PortType PortType;
        PortType* port = new PortType(it->name);
        ports()->addPort(it->name, *port);
        typename MappingType::value_type din = {*it, port};
        mapping.push_back(din);
    }
}

void Task::createUARTPorts()
{
    UARTsConfig uarts_config = _uarts.get();
    for(UARTsConfig::iterator it = uarts_config.begin(); it != uarts_config.end(); ++it) {
        // Creating Orogen component ports
        OutputPort<iodrivers_base::RawPacket>* new_output_port =
            new OutputPort<iodrivers_base::RawPacket>(it->name);
        InputPort<iodrivers_base::RawPacket>* new_input_port =
            new InputPort<iodrivers_base::RawPacket>("w" + it->name);
        ports()->addPort(new_output_port->getName(), *new_output_port);
        ports()->addPort(new_input_port->getName(), *new_input_port);
        UART config = {true, *it, new_output_port, new_input_port};
        uarts[it->uart_module] = config;
    }
}

static void displayAllDevices(SR_IPLIST* list)
{
    log(Warning) << "Devices found: \n";
    while (list != NULL)
    {
        log(Warning)
            << "  ip(" << ipv4ToString(list->ip) << ")"
            << "  mac(" << macToString(list->mac) << ")" << endlog();
        list = list->next;	
    }
}

string findDeviceIP(string const& bcast_addr, string const& mac, int port)
{
    SR_IPLIST *list = sr_discover(bcast_addr.c_str(), port);
    if (!list)
    {  
        log(Error) << "There are no devices on " << bcast_addr << endlog();
        return string();
    }

    SR_IPLIST *l = list;
    string lcase_mac = boost::to_lower_copy(mac);
    while (l != NULL && macToString(l->mac) != lcase_mac)
        l = l->next;

    if (!l)
    {
        log(Error) << mac << " device not found." << endlog();
        displayAllDevices(list);
        sr_discover_free(list);
        return string();
    }

    string ip = ipv4ToString(l->ip);
    log(Info) << "Found device with IP " << ip << endlog();
    sr_discover_free(list);
    return ip;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // Auto discovery or explicit IP
    string ip_addr;
    if (_mac.get().empty())
        ip_addr = _ip.get();
    else
        ip_addr = findDeviceIP(_ip.get(), _mac.get(), _port.get());

    srh = sr_open_eth(ip_addr.c_str(), _port.get());
    if (!srh)
    {
        log(Error) << "could not open device at " << ip_addr << ":" << _port.get() << endlog();
        return false;
    }

    try
    {
        resetHardware();
        setupHardware();
        createDigitalPorts(_digital_ins.get(), din_mapping);
        createDigitalPorts(_digital_outs.get(), dout_mapping);
        createUARTPorts();
    }
    catch(...)
    {
        sr_close(srh);
    }
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    digitalOutState = 0;
    return true;
}

void Task::readDin()
{
    unsigned short state;
    if(!sr_pins_get(srh,&state)){
        log(Warning) << "cannot read digital pins" << endlog();
        return exception(DIGITAL_IN_READ_ERROR);
    }

    raw_io::Digital sample = { ::base::Time::now(), false };
    for(vector<Din>::iterator it = din_mapping.begin(); it != din_mapping.end(); ++it) {
        sample.data = (state >> it->pinconfig.pin) & 0x1;
        (it->port)->write(sample);
    }
}

void Task::writeDout()
{
    unsigned short newState = digitalOutState;
    for(vector<Dout>::iterator it = dout_mapping.begin(); it != dout_mapping.end(); ++it) {
        raw_io::Digital sample;
        if ((it->port)->read(sample) == NewData)
        {
            if (sample.data)
                newState |= 1 << it->pinconfig.pin;
            else newState &= ~(1 << it->pinconfig.pin);
        }
    }
    if (newState == digitalOutState)
        return;

    if (!sr_pins_set(srh, newState)){
        log(Warning) << "cannot write digital pins" << endlog();
        return exception(DIGITAL_OUT_WRITE_ERROR);
    }
    digitalOutState = newState;
}

void Task::readUART(int uart_module, OutputPort<iodrivers_base::RawPacket>& port)
{
    //Read UART send through Output port
    unsigned short readByteCount;

    do
    {
        bool result =
            sr_uart_read_arr(srh, uart_module, buffer, UART_BUFFER_MAX, &readByteCount);
        if(!result){
            log(Warning) << "UART from port " << port.getName() << 
                " could not be read. The followin error was received: " << sr_error_info(srh) << 
                endlog();
            return exception(UART_READ_ERROR);
        }
        if (readByteCount > 0){
            iodrivers_base::RawPacket packet;
            packet.time = ::base::Time::now();
            packet.data.assign(buffer, buffer + readByteCount);
            port.write(packet);
        }
    } while(readByteCount > 0);
}

void Task::writeUART(int uart_module, InputPort<iodrivers_base::RawPacket>& port)
{
    iodrivers_base::RawPacket packet;
    while (port.read(packet) == NewData){
        bool result = sr_uart_write_arr(srh, uart_module, &(packet.data[0]), packet.data.size());
        if(!result){
            log(Warning) << "UART from port " << port.getName() << 
                " could not be written. The following error was received: " << sr_error_info(srh) << 
                endlog();
            return exception(UART_WRITE_ERROR);
        }
    }
}
void Task::updateHook()
{
    TaskBase::updateHook();

    readDin();
    writeDout();
    for (int i = 0; i < 2; ++i){
        UART const& uart = uarts[i];
        if (uart.enabled)
        {
            readUART(i, *uart.output);
            writeUART(i, *uart.input);
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

    for(vector<Din>::iterator it = din_mapping.begin(); it != din_mapping.end(); ++it) {
        ports()->removePort(it->port->getName());
        delete it->port;
    }

    for(vector<Dout>::iterator it = dout_mapping.begin(); it != dout_mapping.end(); ++it) {
        ports()->removePort(it->port->getName());
        delete it->port;
    }

    for (int i = 0; i < UART_MODULES_COUNT; ++i)
    {
        ports()->removePort(uarts[i].output->getName());
        ports()->removePort(uarts[i].input->getName());
        delete uarts[i].output;
        delete uarts[i].input;
    }
    resetHardware();
    sr_close(srh);
    srh=NULL;
}
