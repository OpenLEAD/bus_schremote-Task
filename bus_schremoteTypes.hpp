#ifndef bus_schremote_TYPES_HPP
#define bus_schremote_TYPES_HPP

#include <string>
#include <vector>
#include <boost/cstdint.hpp>
#include <iodrivers_base/Status.hpp>

namespace bus_schremote {

    enum UART_MODES {
        UART_MODE_PARITY_NO = 0x0000,
        UART_MODE_PARITY_EVEN = 0x0002,
        UART_MODE_PARITY_ODD = 0x0004,
        UART_MODE_STOP_ONE = 0x0000,
        UART_MODE_STOP_TWO = 0x0001,
        UART_MODE_STD = UART_MODE_PARITY_NO | UART_MODE_STOP_ONE
    };

    enum PIN_TYPES
    {
        PIN_ANALOG_IN,
        PIN_DIN,
        PIN_DIN_PULLUP,
        PIN_DOUT_LOW,
        PIN_DOUT_HIGH,
        PIN_DOUT_OPENDRAIN_OPEN,
        PIN_DOUT_OPENDRAIN_SHORT
    };

    /** Configuration of a digital I/O
     *
     * It is used for both input and output
     */
    struct PinConfig
    {
        /** The pin index
         *
         * It is zero based
         */
        unsigned int pin;
        /** The pin type
         */
        PIN_TYPES type;
        /** The I/O name
         *
         * The port created on the task interface will be named after this. For
         * instance, if 'name' is 'pressure' for a digital input, an input port
         * with the name 'pressure' will be created on the task interface. If it
         * is for a digital output, the port will be an output port.
         */
        std::string name;

        PinConfig()
            : pin(-1)
            , type(PIN_DIN) {}
    };
    typedef std::vector<PinConfig> PinsConfig;

    /** Configuration of an UART module */
    struct UARTConfig
    {
        UARTConfig()
            : uart_module(0)
            , mode(UART_MODE_STD)
            , tx_type(PIN_DOUT_OPENDRAIN_OPEN)
            , baud(9600)
            , enable_send(-1)
        {}
        /** The index of the UART module that should be configured.
         *
         * It is zero-based
         */
        unsigned int uart_module;
        /** The UART mode as an OR-based field of values in UART_MODES
         *
         * Defaults to UART_MODE_STD
         */
        int mode; 

        /** The GPIO pin that should be used for TX
         *
         * Index is 0 based
         */
        int tx;
        /** In which mode the TX pin should be set. It has to be one of the DOUT
         * types
         */
        PIN_TYPES tx_type;
        /** The GPIO pin that should be used for RX
         *
         * Index is 0 based
         */
        int rx;
        /** The baud rate
         */
        int baud;
        /** The name of the UART
         *
         * The ports created on the task interface will be named after this. For
         * instance, if 'name' is 'pressure', the task's output port (on which
         * will be sent the data *sent* by the pressure sensor) will be named
         * 'pressure', and the task's input port (on which will be received the
         * data that should be sent to the pressure sensor) will be named
         *
         * In addition, a pressure_stats port of type iodrivers_base/Status is
         * added to give statistics about the I/O on this port.
         */
        std::string name;

        // HACK FOR ROSA
        //
        // If positive, the pin listed here will be set to 1 when we send
        // some stuff on the UART. In addition, the component will reset it
        // to zero when we estimate that the data has probably been sent
        //
        // The pin needs to also be configured as a digital input
        int enable_send;
    };
    typedef std::vector<UARTConfig> UARTsConfig;

    struct UARTStatus : public iodrivers_base::Status
    {
        UARTStatus()
            : overflow(0) {}

        /** How many times the internal UART RX buffer could have overflowed */
        boost::uint16_t overflow;
    };
}

#endif

