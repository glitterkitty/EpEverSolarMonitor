# EpEverSolarMonitor

## Purpose

This code is for reading live-, statistical and status-data from
an EpEver LandStar B ( LS1024B ) via a Modbus connection.

The data is then published via mqtt to be fed to gafana and pimatic.

## Description

This code started as a small sketch to read the data via modbus and 
then got additions for mqtt, deep-sleep, debug a.t.l.. It got quite big 
an should get some restructuring...

If you have another EpEver charge controller (like a Tracer), 
you may need to adjust the register/data locations according 
to the datasheet.

## Hardware

I'm using a NodeMCU clone (Board: Lolin Wemos D1 R2 & mini) and a 
widespread MAX485 breakout module to connect to the RJ45 port of the 
solar charge controller.

Both modules are powered using the (in my case) 7.5 Volt supply-voltage
that is available at the RJ45 port. If you're using another esp module 
(e.g. Wemos D1 mini), make sure, the onboard voltage-regulator can 
handle the 7.5 volts from the EpEver.

To avoid the need of a level-shifter, the max485 module is powered only
with 3V3 from the NodeMCU, which works for me, but YMMV.

Power-consumption is roughly 4mA during Deep-Sleep, mostly due to the 
onboard leds, I guess. When running, the power-demand gets up to about 
75mA for 3-4 seconds. 


## Connections
    
    MAX485         NodeMCU 
        DI              TX
        DE              D2
        RE              D1
        RO              RX
        VCC             3V3 !!!
        GND             GND
        


    EpEver RJ45                        MAX485      NodeMCU
    pin1  +7.5 V       org-wht                       Vin
    pin8  GND          brn                           GND
    pin6  RS-485-A     grn               A
    pin4  RS-485-B     blu               B


connect DE (Max485) with a pull-down resistor (e.g. 6k7) to GND,
to hold that line down in Deep-Sleep to lower power consumption

connect D0 (NodeMCU) with reset (NodeMCU) for DeepSleep wake-up to work

connect D6 (NodeMCU)and D7 (NodeMCU) to enable debug-mode. this 
sets the sleep duration to only 10 seconds

some datasheets list different pinouts for the RJ45 jack!  swap A<->B if 
connection fails. Check voltage-level and -polarity before connecting!


## Note

I'm excessively using the union-struct trick to map buffer-data 
to structs here. Most of the defines for the data-locations
are for reference only and not actually used in the code.

Some parts are not done yet, or not done properly, like the switching of the load or the evaluation of the charging-equipment status. 

## Attribution

I got loads of info for this from:

    [https://www.eevblog.com/forum/projects/nodemcu-esp8266-rs485-epever-solar-monitor-diy/](https://www.eevblog.com/forum/projects/nodemcu-esp8266-rs485-epever-solar-monitor-diy/)
    [https://www.eevblog.com/forum/projects/nodemcu-esp8266-rs485-epever-solar-monitor-diy/](http://4-20ma.io/ModbusMaster)


## Furthermore

For taking the data to grafana, you might want to take a look here: 

    (https://github.com/glitterkitty/mqtt-mysql)


Have Phun!!!
