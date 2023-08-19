onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib openram8_opt

do {wave.do}

view wave
view structure
view signals

do {openram8.udo}

run -all

quit -force
