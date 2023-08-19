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

module openram8_tb;

// map_ram Parameters
parameter PERIOD        = 10     ;

// map_ram Inputs
reg   clk        =0          ;
reg ena;
reg enb;
reg   wea         ;
reg   [7:0] dina             ;
reg   [11:0]  addra  ;
reg   [11:0]  addrb  ;

// map_ram Outputs
wire   [7:0] douta             ;
wire   [7:0] doutb             ;


initial
begin
    forever #(PERIOD/2)  clk=~clk;
end

openram8 your_instance_name (
  .clka(clk),    // input wire clka
  .ena(ena),      // input wire ena
  .wea(wea),      // input wire [0 : 0] wea
  .addra(addra),  // input wire [11 : 0] addra
  .dina(dina),    // input wire [7 : 0] dina
  .clkb(clk),    // input wire clkb
  .enb(enb),      // input wire enb
  .addrb(addrb),  // input wire [11 : 0] addrb
  .doutb(doutb)  // output wire [7 : 0] doutb
);

initial
begin
    //1
    #(PERIOD*5) 
    ena = 1;
    wea = 1;//д
    addra = 1;
    dina = 1;
    enb=0;
    addrb=0;
                        
                       #(PERIOD) 
                       ena =1;
                       wea = 1;//д
                        addra = 2;
                        dina = 2;   
                        enb=1;
                        addrb =1; 
                        
                        #(PERIOD) 
                        ena = 1;
                        wea = 1;//д
                        addra = 1;
                        dina = 3;    
                        enb=0;
                        addrb=2;
                                  
                                     #(PERIOD) 
                                     ena = 1;
                                     wea = 1;//д
                        addra = 2;
                        dina = 4;   
                        enb=1;
                        addrb=2;
                         
                              #(PERIOD) 
                                     ena = 0;
                                             wea = 1;//д
                        addra = 0;
                        dina = 5;    
                        enb=0;
                        addrb=0;
     
                        
    $stop;
end

endmodule
