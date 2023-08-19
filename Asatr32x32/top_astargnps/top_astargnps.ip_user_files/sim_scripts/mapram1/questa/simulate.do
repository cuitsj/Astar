onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib mapram1_opt

do {wave.do}

view wave
view structure
view signals

do {mapram1.udo}

run -all

quit -force
