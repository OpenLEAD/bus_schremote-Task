require 'orocos'
require 'readline'

include Orocos
Orocos.initialize


Orocos.run 'bus_schremote::Task' => 'bus_schremote',
    'profundimetro::Task' => 'profundimetro' do  
  
  bus_schremote = Orocos.name_service.get 'bus_schremote'
  profundimetro = Orocos.name_service.get 'profundimetro'
  
  # Never assume that a component supports being reconnected
  # at runtime, it might not be the case
  #
  # If you have the choice, connect before the component is
  # even configured
  
  bus_schremote.ip = '192.168.1.255' #'192.168.1.191'
  bus_schremote.mac = '00:04:A3:31:00:14'
  #bus_schremote.digital_ins = [{:pin => 9, :name => 'induc1'},{:pin => 10, :name => 'induc2'},{:pin => 10, :name => 'induc22'}]
  #bus_schremote.digital_ins = [{:pin => 9, :name => 'inducin'}]
  bus_schremote.digital_outs = [{:pin => 8, :name => 'sendout'}]
  bus_schremote.uarts = [{:uart_module => 0, :mode => 3, :tx => 4, :rx => 5, :baud => 115200, :name => 'profund'}]
  # bus_schremote.uarts = [{:uart_module => 0, :mode => 3, :tx => 4, :rx => 5, :baud => 115200, :name => 'uart0'}, {:uart_module => 1, :mode => 3, :tx => 10, :rx => 9, :baud => 115200, :name => 'uart1'}]
  # bus_schremote.uarts = [{:uart_module => 0, :mode => 0, :tx => 4, :rx => 0, :baud => 115200, :name => 'uart0'}]
  bus_schremote.configure

  profundimetro.send485.connect_to bus_schremote.sendout
  profundimetro.outputRaw.connect_to bus_schremote.profund_in
  bus_schremote.profund_out.connect_to profundimetro.inputRaw


  profundimetro.configure
  bus_schremote.start
  profundimetro.start

 # while true
  #  sleep 1
 # end
  #while true
    #uart0_in.write(msg_in)
   # sleep 1
  #end 

  Readline::readline("Press ENTER to exit\n") do
  end
  

end

