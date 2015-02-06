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
        PinsConfig const& config) const
{
    for(PinsConfig::const_iterator it = config.begin(); it != config.end(); ++it){
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
    sr_spi_disable(srh);
    for (int i = 0; i < I2C_MODULES_COUNT; ++i)
        sr_i2c_disable(srh, i);
    for (int i = 0; i < CNT_MODULES_COUNT; ++i)
        sr_cnt_disable(srh, i);
}

static sr_pin_type pinTypeToSDK(PIN_TYPES type)
{
    switch (type)
    {
        case PIN_ANALOG_IN: return sr_pt_analog_in;
        case PIN_DIN: return sr_pt_din;
        case PIN_DIN_PULLUP: return sr_pt_din_pullup;
        case PIN_DOUT_LOW: return sr_pt_dout_low;
        case PIN_DOUT_HIGH: return sr_pt_dout_high;
        case PIN_DOUT_OPENDRAIN_SHORT: return sr_pt_dout_opendrain_short;
        case PIN_DOUT_OPENDRAIN_OPEN: return sr_pt_dout_opendrain_open;
        default:
            throw std::logic_error("pin type not handled in switch in pinTypeToSDK");
    };
}

void Task::setupHardware()
{
    PinsConfig pins[3] = { _digital_ins.get(), _digital_outs.get(), _analog_ins.get() };
    for(int i = 0; i < 3; ++i){
        for(PinsConfig::iterator it = pins[i].begin(); it != pins[i].end(); ++it){
            sr_pin_type pin_type = pinTypeToSDK(it->type);
            if(!sr_pin_setup(srh,it->pin,pin_type))
                throw runtime_error("cannot setup pin " + lexical_cast<string>(it->pin) + ": " + sr_error_info(srh));
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


        sr_pin_type tx_type = pinTypeToSDK(it->tx_type);
        if(!sr_pin_setup(srh,it->tx,tx_type)){
            throw logic_error("TX pin " + lexical_cast<string>(it->tx) +
                    " configured to be used for UART " + lexical_cast<string>(it->uart_module) +
                    " could not be set up: " + string(sr_error_info(srh)));
        }

        if(!sr_uart_enable(srh, it->uart_module, it->mode, it->baud, it->rx, it->tx)){
            throw logic_error("could not enable UART module " + lexical_cast<string>(it->uart_module) + ": " + sr_error_info(srh));
        }
    }
}

template<typename MappingType>
void Task::createPinsPorts(PinsConfig const& config, MappingType& mapping)
{
    for(PinsConfig::const_iterator it = config.begin(); it != config.end(); ++it) {
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
        OutputPort<UARTStatus>* status_port =
            new OutputPort<UARTStatus>(it->name + "_status");
        ports()->addPort(new_output_port->getName(), *new_output_port);
        ports()->addPort(new_input_port->getName(), *new_input_port);
        ports()->addPort(status_port->getName(), *status_port);

        UART& config = uarts[it->uart_module];
        config.enabled = true;
        config.config = *it;
        config.status = UARTStatus();
        config.output = new_output_port;
        config.input = new_input_port;
        config.status_port = status_port;
        config.packet.data.reserve(UART_BUFFER_MAX);
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
        throw std::runtime_error("could not find device with MAC " + mac + " on broadcast address " + bcast_addr);
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
        createPinsPorts(_digital_ins.get(), din_mapping);
        createPinsPorts(_digital_outs.get(), dout_mapping);
        createPinsPorts(_analog_ins.get(), analog_mapping);
        createUARTPorts();
    }
    catch(...)
    {
        sr_close(srh);
        throw;
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

void Task::readAnalog()
{
    for(vector<Analog>::iterator it = analog_mapping.begin(); it != analog_mapping.end(); ++it) {
        Analog::PortType& port = *(it->port);
        if (!port.connected())
            continue;

        int pin = it->config.pin;
        unsigned short data;
        if (!sr_pin_get_analog(srh, pin, &data))
        {
            log(Warning) << "cannot read analog pin " << pin << ": " << sr_error_info(srh) << endlog();
            return exception(ANALOG_IN_READ_ERROR);
        }

        raw_io::Analog sample ( ::base::Time::now(), 0 );
        sample.data = it->config.analog_scale_factor *
            _reference_voltage.get() * static_cast<float>(data) / 1024;
        port.write(sample);
    }
}

void Task::readDin()
{
    for(vector<Din>::iterator it = din_mapping.begin(); it != din_mapping.end(); ++it) {
        Din::PortType& port = *(it->port);
        if (!port.connected())
            continue;

        raw_io::Digital sample (::base::Time::now(), false);
        int pin = it->config.pin;
        if (!sr_pin_get(srh, pin, &sample.data))
        {
            log(Warning) << "cannot read digital pin " << pin << ": " << sr_error_info(srh) << endlog();
            return exception(DIGITAL_IN_READ_ERROR);
        }

        port.write(sample);
    }
}

void Task::writeDout()
{
    for(vector<Dout>::iterator it = dout_mapping.begin(); it != dout_mapping.end(); ++it) {
        raw_io::Digital sample;
        if ((it->port)->read(sample) == NewData)
        {
            if (!sr_pin_set(srh, it->config.pin, sample.data)){
                log(Warning) << "cannot write digital pin " << it->config.pin << ": " << sr_error_info(srh) << endlog();
                return exception(DIGITAL_OUT_WRITE_ERROR);
            }
        }
    }
}

int Task::readUART(int uart_module, UART& uart)
{
    boost::uint8_t buffer[UART_BUFFER_MAX];

    unsigned short readByteCount;
    bool result =
        sr_uart_read_arr(srh, uart_module, buffer, UART_BUFFER_MAX, &readByteCount);
    if(!result){
        log(Warning) << "could not read data from UART " << uart.config.name << ": "
            << sr_error_info(srh) << endlog();
        exception(UART_READ_ERROR);
        return 0;
    }


    if (readByteCount > 0){
        iodrivers_base::RawPacket& packet(uart.packet);
        packet.time = ::base::Time::now();
        packet.data.assign(buffer, buffer + readByteCount);
        uart.output->write(packet);

        uart.status.stamp = ::base::Time::now();
        uart.status.good_rx += readByteCount;
        if (readByteCount == UART_BUFFER_MAX)
        {
            uart.status.overflow++;
            if (_fail_on_uart_buffer_overflow.get())
                exception(UART_READ_BUFFER_OVERFLOW);
        }
        uart.status_port->write(uart.status);
    }

    return readByteCount;
}

int Task::writeUART(int uart_module, UART& uart)
{
    int totalCount = 0;
    iodrivers_base::RawPacket packet;
    while (uart.input->read(packet) == NewData){
        int enable_send = uart.config.enable_send;
        int timeout_ms = packet.data.size() * 1000 * 8 / uart.config.baud;
        if (enable_send != -1)
        {
            if (!sr_pin_set(srh, enable_send, true))
            {
                log(Warning) << "failed to set the enable send pin " << enable_send << " for UART " << uart.config.name << ": " << sr_error_info(srh) << endlog();
                exception(DIGITAL_OUT_WRITE_ERROR);
                return 0;
            }
        }

        bool result = sr_uart_write_arr(srh, uart_module, &(packet.data[0]), packet.data.size());

        if (enable_send != -1)
        {
            usleep(timeout_ms * 1000);
            if (!sr_pin_set(srh, enable_send, false))
            {
                log(Warning) << "failed to set the enable send pin " << enable_send << " for UART " << uart.config.name << ": " << sr_error_info(srh) << endlog();
                exception(DIGITAL_OUT_WRITE_ERROR);
                return 0;
            }
        }

        if(!result){
            log(Warning) << "could not write data to UART " << uart.config.name << ": "
                << sr_error_info(srh) << endlog();
            exception(UART_WRITE_ERROR);
            return 0;
        }
        uart.status.tx += packet.data.size();
        uart.status.stamp = ::base::Time::now();
        uart.status_port->write(uart.status);
        totalCount += packet.data.size();
    }
    return totalCount;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    // Read all UARTs first. This is where there is a very limited hardware
    // buffer, so empty it as soon as possible
    for (int i = 0; i < 2; ++i){
        UART& uart = uarts[i];
        if (uart.enabled)
            readUART(i, uart);
    }

    // See comment above explaining why reading and writing are separated
    for (int i = 0; i < 2; ++i){
        UART& uart = uarts[i];
        if (uart.enabled)
            writeUART(i, uart);
    }

    readDin();
    writeDout();
    readAnalog();
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
    din_mapping.clear();

    for(vector<Dout>::iterator it = dout_mapping.begin(); it != dout_mapping.end(); ++it) {
        ports()->removePort(it->port->getName());
        delete it->port;
    }
    dout_mapping.clear();

    for (int i = 0; i < UART_MODULES_COUNT; ++i)
    {
        if (!uarts[i].enabled)
            continue;

        ports()->removePort(uarts[i].output->getName());
        ports()->removePort(uarts[i].input->getName());
        ports()->removePort(uarts[i].status_port->getName());
        delete uarts[i].output;
        delete uarts[i].input;
        delete uarts[i].status_port;
        uarts[i].enabled = false;
    }
    resetHardware();
    sr_close(srh);
    srh=NULL;
}
