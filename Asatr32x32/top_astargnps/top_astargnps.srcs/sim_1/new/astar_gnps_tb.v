`timescale  1ns / 1ns

module astar_gnps_tb;

// gnps Parameters
parameter PERIOD        = 10     ;
parameter MAP_ROW       = 8'd32;
parameter MAP_COLUMN    = 8'd32 ;
parameter MAP_EXP       = 8'd5;
parameter START_ROW     = 8'd1;
parameter START_COLUMN  = 8'd1 ;
parameter END_ROW       = 8'd29 ;
parameter END_COLUMN    = 8'd29;

// gnps Inputs
reg   clk                                  = 0 ;
 reg   clk2                                  = 0 ;
reg   rst_n                                = 0 ;
reg   rxprocess_en                         = 0 ;
reg   [7:0]  rx_data                       = 0 ;
reg   tx_done                              = 1 ;

// gnps Outputs
wire  [7:0]  tx_data                       ;
wire  tx_flag                              ;
wire  led0                                 ;
wire  led1                                 ;


initial
begin
    forever #(PERIOD/2)  clk=~clk;
end

 initial
 begin
    #(PERIOD/2)
     forever #(PERIOD/2)  clk2=~clk2;
 end

initial
begin
    #(PERIOD*2) rst_n  =  1;
end

astar_gnps #(
    .MAP_ROW      ( MAP_ROW      ),
    .MAP_COLUMN   ( MAP_COLUMN   ),
    .MAP_EXP      ( MAP_EXP      ),
    .START_ROW    ( START_ROW    ),
    .START_COLUMN ( START_COLUMN ),
    .END_ROW      ( END_ROW      ),
    .END_COLUMN   ( END_COLUMN   ))
 u_astar_gnps (
    .clk                     ( clk                 ),
     .clk2                     ( clk2                 ),
    .rst_n                   ( rst_n               ),
    .rxprocess_en            ( rxprocess_en        ),
    .rx_data                 ( rx_data       [7:0] ),
    .tx_done                 ( tx_done             ),

    .tx_data                 ( tx_data       [7:0] ),
    .tx_flag                 ( tx_flag             ),
    .led0                    ( led0                ),
    .led1                    ( led1                )
);
integer      i ;
initial
begin
    //1
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;

    //1
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;



    for (i=0; i<=28; i=i+1) begin
        if (i==3)begin
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'hFF;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h00;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h00;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h40;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
        end
        else begin
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h01;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h00;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h00;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
            #(PERIOD*8);
            rxprocess_en = 1;
            rx_data = 8'h40;
            #(PERIOD*8);
            rxprocess_en = 0;
            rx_data = 0;
        end
    end
    //31       
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'hFF;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;

    //32
    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'h01;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;

    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'h00;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;

    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'h00;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;

    #(PERIOD*8) rxprocess_en = 1;
                        rx_data = 8'h40;
    #(PERIOD*8) rxprocess_en = 0;
                        rx_data = 0;             
     
                        
    $stop;
end

endmodule