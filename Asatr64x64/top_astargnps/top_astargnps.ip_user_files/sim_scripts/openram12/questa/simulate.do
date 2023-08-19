onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib openram12_opt

do {wave.do}

view wave
view structure
view signals

do {openram12.udo}

run -all

quit -force
