require 'orocos'
require 'readline'

include Orocos
Orocos.initialize


Orocos.run 'bus_schremote::Task' => 'bus_schremote' do  
  
  bus_schremote = Orocos.name_service.get 'bus_schremote'
  
  # Never assume that a component supports being reconnected
  # at runtime, it might not be the case
  #
  # If you have the choice, connect before the component is
  # even configured
  
  bus_schremote.ip = '192.168.1.255' #'192.168.1.191'
  bus_schremote.mac = '00:04:A3:31:00:14'
  # bus_schremote.digital_ins = [{:pin => 9, :name => 'induc1'},{:pin => 10, :name => 'induc2'},{:pin => 10, :name => 'induc22'}]
  bus_schremote.uarts = [{:uart_module => 0, :mode => 3, :tx => 4, :rx => 5, :baud => 115200, :name => 'uart0'}, {:uart_module => 1, :mode => 3, :tx => 10, :rx => 9, :baud => 115200, :name => 'uart1'}]

  bus_schremote.configure

  uart0_in = bus_schremote.uart0_in.writer
  uart0_out = bus_schremote.uart0_out.reader

  uart1_in = bus_schremote.uart1_in.writer
  uart1_out = bus_schremote.uart1_out.reader

  msg_in = uart0_in.new_sample
  msg_in.time = Types::Base::Time.new
  msg_in.data = [1,4,5]


  bus_schremote.start
  
  while true
    uart0_in.write(msg_in)
    sleep 1
  end 

  Readline::readline("Press ENTER to exit\n")
  

end

