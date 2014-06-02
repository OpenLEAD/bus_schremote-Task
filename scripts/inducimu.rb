require 'orocos'
require 'readline'

include Orocos
Orocos.initialize


Orocos.run 'bus_schremote::Task' => 'bus_schremote',
	'spatial::Task' => 'spatial' do    
  
  bus_schremote = Orocos.name_service.get 'bus_schremote'
  spatial = Orocos.name_service.get 'spatial'
  
  # Never assume that a component supports being reconnected
  # at runtime, it might not be the case
  #
  # If you have the choice, connect before the component is
  # even configured
  
  bus_schremote.ip = '192.168.1.255' #'192.168.1.191'
  bus_schremote.mac = '00:04:A3:31:00:12'
  #bus_schremote.digital_ins = [{:pin => 9, :name => 'induc1'},{:pin => 10, :name => 'induc2'},{:pin => 10, :name => 'induc22'}]
  bus_schremote.digital_ins = [{:pin => 10, :name => 'induc2'},{:pin => 11, :name => 'induc3'}]
  bus_schremote.uarts = [{:uart_module => 0, :mode => 0, :tx => 4, :rx => 0, :baud => 115200, :name => 'imu'}]
  #bus_schremote.digital_outs = [{:pin => 4, :name => 'inducout'}]
  # bus_schremote.uarts = [{:uart_module => 0, :mode => 3, :tx => 4, :rx => 5, :baud => 115200, :name => 'uart0'}, {:uart_module => 1, :mode => 3, :tx => 10, :rx => 9, :baud => 115200, :name => 'uart1'}]
  # bus_schremote.uarts = [{:uart_module => 0, :mode => 0, :tx => 4, :rx => 0, :baud => 115200, :name => 'uart0'}]
  bus_schremote.configure



  bus_schremote.imu_out.connect_to spatial.com_input
  spatial.configure

  bus_schremote.start
  spatial.start

  Readline::readline("Press ENTER to exit\n") do
  end
  

end

