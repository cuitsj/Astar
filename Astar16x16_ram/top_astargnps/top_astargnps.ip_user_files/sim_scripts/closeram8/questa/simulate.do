onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib closeram8_opt

do {wave.do}

view wave
view structure
view signals

do {closeram8.udo}

run -all

quit -force
