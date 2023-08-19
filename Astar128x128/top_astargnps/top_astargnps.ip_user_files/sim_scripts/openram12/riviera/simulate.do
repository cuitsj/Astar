onbreak {quit -force}
onerror {quit -force}

asim -t 1ps +access +r +m+openram12 -L xil_defaultlib -L xpm -L blk_mem_gen_v8_4_2 -L unisims_ver -L unimacro_ver -L secureip -O5 xil_defaultlib.openram12 xil_defaultlib.glbl

do {wave.do}

view wave
view structure

do {openram12.udo}

run -all

endsim

quit -force
