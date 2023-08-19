`timescale 1ns / 1ns
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/05/16 19:44:21
// Design Name: 
// Module Name: map_ram_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module map_ram_tb;

// map_ram Parameters
parameter PERIOD        = 10     ;

// map_ram Inputs
reg   clk        =0          ;
reg ena;
reg   wea         ;
reg   dina             ;
reg   [11:0]  addra  ;

// map_ram Outputs
wire  douta                       ;


initial
begin
    forever #(PERIOD/2)  clk=~clk;
end


map_ram your_instance_name (
  .clka(clk),    // input wire clka
  .ena(ena),
  .wea(wea),      // input wire [0 : 0] wea
  .addra(addra),  // input wire [11 : 0] addra
  .dina(dina),    // input wire [0 : 0] dina
  .douta(douta)  // output wire [0 : 0] douta
);

initial
begin
    //1
    #(PERIOD*5) 
    ena = 1;
    wea = 1;//д
                        addra = 1;
                        dina = 1;
                        
                       #(PERIOD) 
                       ena =1;
                       wea = 1;//д
                        addra = 2;
                        dina = 0;    
                        
                        #(PERIOD) 
                        ena = 1;
                        wea = 0;//д
                        addra = 1;
                        dina = 1;    
                                  
                                     #(PERIOD) 
                                     ena = 1;
                                     wea = 0;//д
                        addra = 2;
                        dina = 0;    
                              #(PERIOD-PERIOD/2) 
                                     ena = 0;
                                             wea = 0;//д
                        addra = 0;
                        dina = 0;    
     
                        
    $stop;
end

endmodule
