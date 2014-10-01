/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef BUS_SCHREMOTE_TASK_TASK_HPP
#define BUS_SCHREMOTE_TASK_TASK_HPP

#include "bus_schremote/TaskBase.hpp"
#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>
#include <bus_schremoteTypes.hpp>
#include <stdio.h>
#include "tasks/libschremote.h"

namespace bus_schremote {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','bus_schremote::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	SR_HANDLE srh;
        static const int UART_MODULES_COUNT = 2;
        static const int I2C_MODULES_COUNT = 2;
        static const int CNT_MODULES_COUNT = 4;
        static const int NUMBER_OF_PINS = 12;
	static const bool AnIn[NUMBER_OF_PINS];
	static const bool I2C[NUMBER_OF_PINS];
	static const bool UART_SPI_CNT[NUMBER_OF_PINS];
        static const unsigned short  UART_BUFFER_MAX = 256u;

        unsigned short digitalOutState;

        void validateDigitalIOConfiguration(
                std::set<std::string>& portNames,
                std::vector<bool>& usedPins,
                PinsConfig const& config) const;
        void validateUARTPin(std::vector<bool> const& usedPins, unsigned int pin, unsigned int module);
        void validateUARTConfiguration(
                std::set<std::string>& portNames,
                std::vector<bool>& usedPins,
                UARTsConfig const& uarts);
        void validateConfiguration();
        void resetHardware();
        void setupHardware();

        template<typename MappingType>
        void createPinsPorts(PinsConfig const& config, MappingType& mapping);
        void createUARTPorts();

        void readDin();
        void writeDout();
        struct UART;
        int readUART(int uart_module, UART& uart);
        int writeUART(int uart_module, UART& uart);

	struct Din
	{
            typedef RTT::OutputPort<raw_io::Digital> PortType;
            PinConfig config;
            PortType* port;
	};
	std::vector<Din> din_mapping;

	struct Dout
	{
            typedef RTT::InputPort<raw_io::Digital> PortType;
            PinConfig config;
            PortType* port;
	};
	std::vector<Dout> dout_mapping;

        struct Analog
        {
            typedef RTT::OutputPort<raw_io::Analog> PortType;
            PinConfig config;
            PortType* port;
        };
	std::vector<Analog> analog_mapping;

	struct UART
	{
            bool enabled;
            UARTConfig config;
            UARTStatus status;
            RTT::OutputPort<iodrivers_base::RawPacket>* output;
            RTT::InputPort<iodrivers_base::RawPacket>* input;
            RTT::OutputPort<UARTStatus>* status_port;
            iodrivers_base::RawPacket packet;
	};
        UART uarts[2];

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "bus_schremote::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

