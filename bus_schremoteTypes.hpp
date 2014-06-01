#ifndef bus_schremote_TYPES_HPP
#define bus_schremote_TYPES_HPP

#define UART_MODE_STD 0x0003 // UART_MODE_PARITY_EVEN | UART_MODE_STOP_TWO

#include <string>
#include <vector>
/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace bus_schremote {

enum {
	UART_MODE_PARITY_NO = 0x0000,
	UART_MODE_PARITY_EVEN = 0x0002,
	UART_MODE_PARITY_ODD = 0x0004,
	UART_MODE_STOP_ONE = 0x0000,
	UART_MODE_STOP_TWO = 0x0001
};

struct DConfig
{
        int pin;
	std::string name;
};

typedef std::vector<DConfig> DsConfig;

struct UARTConfig
{
	UARTConfig(): baud(9600), mode(UART_MODE_STD), uart_module(0) {}
	int uart_module;
	int mode; 
        int tx;
        int rx;
	int baud;
	std::string name;

};

typedef std::vector<UARTConfig> UARTsConfig;

}

#endif

