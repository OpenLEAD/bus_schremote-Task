#ifndef bus_schremote_TYPES_HPP
#define bus_schremote_TYPES_HPP

#include <string>
#include <vector>
/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace bus_schremote {

    enum UART_MODES {
        UART_MODE_PARITY_NO = 0x0000,
        UART_MODE_PARITY_EVEN = 0x0002,
        UART_MODE_PARITY_ODD = 0x0004,
        UART_MODE_STOP_ONE = 0x0000,
        UART_MODE_STOP_TWO = 0x0001,
        UART_MODE_STD = UART_MODE_PARITY_NO | UART_MODE_STOP_ONE
    };

    struct DConfig
    {
        unsigned int pin;
        std::string name;
    };
    typedef std::vector<DConfig> DsConfig;

    struct UARTConfig
    {
        UARTConfig()
            : uart_module(0)
            , mode(UART_MODE_STD)
            , baud(9600)
        {}
        unsigned int uart_module;
        int mode; 
        int tx;
        int rx;
        int baud;
        std::string name;
    };
    typedef std::vector<UARTConfig> UARTsConfig;

}

#endif

