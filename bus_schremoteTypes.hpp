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

struct PinConfig
{
        int pin;
	std::string name;

};

typedef std::vector<PinConfig> PinsConfig;

struct UARTConfig
{
        int tx;
        int rx;
	std::string name;

};

typedef std::vector<UARTConfig> UARTsConfig;


}

#endif

