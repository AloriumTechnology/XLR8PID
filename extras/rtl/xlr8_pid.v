/////////////////////////////////
// Filename    : xlr8_pid.v
// Author      : 
// Description : A collection of pid channels and the
//                AVR IO registers needed to access them.
//                Each PID Separately enabled/disabled).
//                Registers
//                  ControlRegister : 
//                        [7]   = enable channel (write, read returns enable with/of selected channel)
//                        [6]   = disable channel (strobe, always read as zero)
//                        [5]   = update channel zero count and yyye (always read as zero)
//                        [4:0] = pid channel to enable/disable/update
//                  KD high and low derivative coeff
//                  KI high and low intergral coeff
//                  KP high and low proportional coeff
//                  SP high and low setpoint - what you want the process variable to be
//                  PV high and low process variable - read this from a sensor and write here
//                  OP high and low output - read and send this to a servo
//
//                   To start a channel typically the channel is reset
//                    first, then the control register with the desired channel indicated and
//                    both the enable and update bits set
//
// Copyright 2017, Superion Technology Group. All Rights Reserved
/////////////////////////////////

module xlr8_pid
 #(parameter NUM_PIDS = 4,
   parameter PIDCR_ADDR  = 6'h0,  // pid control register
   parameter PID_KD_H_ADDR = 6'h0, // pid derrivative
   parameter PID_KD_L_ADDR = 6'h0, // pid derrivative
   parameter PID_KI_H_ADDR = 6'h0, // pid integral
   parameter PID_KI_L_ADDR = 6'h0, // pid integral
   parameter PID_KP_H_ADDR = 6'h0, // pid proportional
   parameter PID_KP_L_ADDR = 6'h0, // pid proportional
   parameter PID_SP_H_ADDR = 6'h0, // pid setpoint
   parameter PID_SP_L_ADDR = 6'h0, // pid setpoint
   parameter PID_PV_H_ADDR = 6'h0, // pid process variable
   parameter PID_PV_L_ADDR = 6'h0, // pid process variable
   parameter PID_OP_H_ADDR = 6'h0, // pid output
   parameter PID_OP_L_ADDR = 6'h0) // output
  (input logic clk,
  input logic                   en1mhz, // clock enable at 1MHz yyye
  input logic                   rstn,
  // Register access for registers in first 64
  input [5:0]                   adr,
  input [7:0]                   dbus_in,
  output [7:0]                  dbus_out,
  input                         iore,
  input                         iowe,
  output wire                   io_out_en,
  // Register access for registers not in first 64
  input wire [7:0]              ramadr,
  input wire                    ramre,
  input wire                    ramwe,
  input wire                    dm_sel,
  // External inputs/outputs
  output logic [NUM_PIDS-1:0]  pids_en
  );

  /////////////////////////////////
  // Local Parameters
  /////////////////////////////////
  localparam NUM_TIMERS = (NUM_PIDS <= 16) ? NUM_PIDS : 16;
  // Registers in I/O address range x0-x3F (memory addresses -x20-0x5F)
  //  use the adr/iore/iowe inputs. Registers in the extended address
  //  range (memory address 0x60 and above) use ramadr/ramre/ramwe
  localparam  PIDCR_DM_LOC     = (PIDCR_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_KD_H_DM_LOC   = (PID_KD_H_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_KD_L_DM_LOC   = (PID_KD_L_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_KI_H_DM_LOC   = (PID_KI_H_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_KI_L_DM_LOC   = (PID_KI_L_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_KP_H_DM_LOC   = (PID_KP_H_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_KP_L_DM_LOC   = (PID_KP_L_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_SP_H_DM_LOC   = (PID_SP_H_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_SP_L_DM_LOC   = (PID_SP_L_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_PV_H_DM_LOC   = (PID_PV_H_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_PV_L_DM_LOC   = (PID_PV_L_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_OP_H_DM_LOC   = (PID_OP_H_ADDR >= 16'h60) ? 1 : 0;
  localparam  PID_OP_L_DM_LOC   = (PID_OP_L_ADDR >= 16'h60) ? 1 : 0;

  localparam PIDEN_BIT   = 7;
  localparam PIDDIS_BIT  = 6;
  localparam PIDUP_BIT   = 5;
  localparam PIDCHAN_LSB = 0;    


  /////////////////////////////////
  // Signals
  /////////////////////////////////
  /*AUTOREG*/
  /*AUTOWIRE*/ 
  logic pidcr_sel;
  logic pid_kd_h_sel;
  logic pid_kd_l_sel;
  logic pid_ki_h_sel;
  logic pid_ki_l_sel;
  logic pid_kp_h_sel;
  logic pid_kp_l_sel;
  logic pid_sp_h_sel;
  logic pid_sp_l_sel;
  logic pid_pv_h_sel;
  logic pid_pv_l_sel;
  logic pid_op_h_sel;
  logic pid_op_l_sel;
  logic pidcr_we ;
  logic pid_kd_h_we ;
  logic pid_kd_l_we ;
  logic pid_ki_h_we ;
  logic pid_ki_l_we ;
  logic pid_kp_h_we ;
  logic pid_kp_l_we ;
  logic pid_sp_h_we ;
  logic pid_sp_l_we ;
  logic pid_pv_h_we ;
  logic pid_pv_l_we ;
  logic pidcr_re ;
  logic pid_kd_h_re ;
  logic pid_kd_l_re ;
  logic pid_ki_h_re ;
  logic pid_ki_l_re ;
  logic pid_kp_h_re ;
  logic pid_kp_l_re ;
  logic pid_sp_h_re ;
  logic pid_sp_l_re ;
  logic pid_pv_h_re ;
  logic pid_pv_l_re ;
  logic pid_op_h_re ;
  logic pid_op_l_re ;
  logic [7:0] pidcr_rdata;
  logic [NUM_PIDS-1:0] load_en; // update of shared variables
  logic       PIDEN;
  logic [4:0] PIDCHAN;
  logic [7:0] PID_KD_H; // Derivitive Coefficients
  logic [7:0] PID_KD_L;
  logic [7:0] PID_KI_H; // Integral Coefficients
  logic [7:0] PID_KI_L;
  logic [7:0] PID_KP_H; // Proportional Coefficients
  logic [7:0] PID_KP_L;
  logic [7:0] PID_SP_H; // The setpoint
  logic [7:0] PID_SP_L;
  logic [7:0] PID_PV_H; // The setpoint
  logic [7:0] PID_PV_L;
  logic [4:0] chan_in;  // Index for selected channel access
  // Channel coefficients
  logic signed [15:0] chan_kd [NUM_PIDS-1:0];
  logic signed [15:0] chan_ki [NUM_PIDS-1:0];
  logic signed [15:0] chan_kp [NUM_PIDS-1:0];
  logic signed [15:0] chan_sp [NUM_PIDS-1:0];
  logic signed [15:0] chan_pv [NUM_PIDS-1:0];
  // Channel internal variables
  logic signed [15:0] k1 [NUM_PIDS-1:0];
  logic signed [15:0] k2 [NUM_PIDS-1:0];
  logic signed [15:0] k3 [NUM_PIDS-1:0];
  logic signed [15:0] err [NUM_PIDS-1:0];
  logic signed [15:0] err_1 [NUM_PIDS-1:0];
  logic signed [15:0] err_2 [NUM_PIDS-1:0];
  logic signed [15:0] chan_output [NUM_PIDS-1:0]; // The control output
  logic signed [15:0] last_out [NUM_PIDS-1:0];
  logic [7:0]  chan_output_rdata_l;
  logic [7:0]  chan_output_rdata_h;
  logic [1:0]  state;
  logic [NUM_PIDS-1:0] trigger_pid;
  logic trigger_enable [NUM_PIDS-1:0];
  logic write_pv;

  localparam IDLE = 2'b00;
  localparam ONE_WRITE = 2'b01;
  localparam TWO_WRITE = 2'b10;
  localparam HIGH_LIMIT =  16384;
  localparam LOW_LIMIT  = -16384;



  // Algorithm Timer
  logic [14:0] timercnt_20; // 20ms counter
  logic sample_20;          // 20ms pulse

  /////////////////////////////////
  // Functions and Tasks
  /////////////////////////////////

  /////////////////////////////////
  // Main Code
  /////////////////////////////////

  // Address Decode
  assign pidcr_sel    = PIDCR_DM_LOC     ?  (dm_sel && ramadr == PIDCR_ADDR )    : (adr[5:0] == PIDCR_ADDR[5:0] ); 
  assign pid_kd_h_sel = PID_KD_H_DM_LOC  ?  (dm_sel && ramadr == PID_KD_H_ADDR ) : (adr[5:0] == PID_KD_H_ADDR[5:0] );
  assign pid_kd_l_sel = PID_KD_L_DM_LOC  ?  (dm_sel && ramadr == PID_KD_L_ADDR ) : (adr[5:0] == PID_KD_L_ADDR[5:0] );
  assign pid_ki_h_sel = PID_KI_H_DM_LOC  ?  (dm_sel && ramadr == PID_KI_H_ADDR ) : (adr[5:0] == PID_KI_H_ADDR[5:0] );
  assign pid_ki_l_sel = PID_KI_L_DM_LOC  ?  (dm_sel && ramadr == PID_KI_L_ADDR ) : (adr[5:0] == PID_KI_L_ADDR[5:0] );
  assign pid_kp_h_sel = PID_KP_H_DM_LOC  ?  (dm_sel && ramadr == PID_KP_H_ADDR ) : (adr[5:0] == PID_KP_H_ADDR[5:0] );
  assign pid_kp_l_sel = PID_KP_L_DM_LOC  ?  (dm_sel && ramadr == PID_KP_L_ADDR ) : (adr[5:0] == PID_KP_L_ADDR[5:0] );
  assign pid_sp_h_sel = PID_SP_H_DM_LOC  ?  (dm_sel && ramadr == PID_SP_H_ADDR ) : (adr[5:0] == PID_SP_H_ADDR[5:0] );
  assign pid_sp_l_sel = PID_SP_L_DM_LOC  ?  (dm_sel && ramadr == PID_SP_L_ADDR ) : (adr[5:0] == PID_SP_L_ADDR[5:0] );
  assign pid_pv_h_sel = PID_PV_H_DM_LOC  ?  (dm_sel && ramadr == PID_PV_H_ADDR ) : (adr[5:0] == PID_PV_H_ADDR[5:0] );
  assign pid_pv_l_sel = PID_PV_L_DM_LOC  ?  (dm_sel && ramadr == PID_PV_L_ADDR ) : (adr[5:0] == PID_PV_L_ADDR[5:0] );
  assign pid_op_h_sel = PID_OP_H_DM_LOC  ?  (dm_sel && ramadr == PID_OP_H_ADDR ) : (adr[5:0] == PID_OP_H_ADDR[5:0] );
  assign pid_op_l_sel = PID_OP_L_DM_LOC  ?  (dm_sel && ramadr == PID_OP_L_ADDR ) : (adr[5:0] == PID_OP_L_ADDR[5:0] );
  // Address based register Write Enable
  assign pidcr_we     = pidcr_sel    && (PIDCR_DM_LOC     ?  ramwe : iowe); 
  assign pid_kd_h_we  = pid_kd_h_sel && (PID_KD_H_DM_LOC  ?  ramwe : iowe); 
  assign pid_kd_l_we  = pid_kd_l_sel && (PID_KD_L_DM_LOC  ?  ramwe : iowe); 
  assign pid_ki_h_we  = pid_ki_h_sel && (PID_KI_H_DM_LOC  ?  ramwe : iowe);
  assign pid_ki_l_we  = pid_ki_l_sel && (PID_KI_L_DM_LOC  ?  ramwe : iowe);
  assign pid_kp_h_we  = pid_kp_h_sel && (PID_KP_H_DM_LOC  ?  ramwe : iowe); 
  assign pid_kp_l_we  = pid_kp_l_sel && (PID_KP_L_DM_LOC  ?  ramwe : iowe); 
  assign pid_sp_h_we  = pid_sp_h_sel && (PID_SP_H_DM_LOC  ?  ramwe : iowe); 
  assign pid_sp_l_we  = pid_sp_l_sel && (PID_SP_L_DM_LOC  ?  ramwe : iowe); 
  assign pid_pv_h_we  = pid_pv_h_sel && (PID_PV_H_DM_LOC  ?  ramwe : iowe); 
  assign pid_pv_l_we  = pid_pv_l_sel && (PID_PV_L_DM_LOC  ?  ramwe : iowe); 
  // Address based register Read Enable
  assign pidcr_re     = pidcr_sel    && (PIDCR_DM_LOC     ?  ramre : iore); 
  assign pid_kd_h_re  = pid_kd_h_sel && (PID_KD_H_DM_LOC  ?  ramre : iore); 
  assign pid_kd_l_re  = pid_kd_l_sel && (PID_KD_L_DM_LOC  ?  ramre : iore); 
  assign pid_ki_h_re  = pid_ki_h_sel && (PID_KI_H_DM_LOC  ?  ramre : iore);
  assign pid_ki_l_re  = pid_ki_l_sel && (PID_KI_L_DM_LOC  ?  ramre : iore);
  assign pid_kp_h_re  = pid_kp_h_sel && (PID_KP_H_DM_LOC  ?  ramre : iore); 
  assign pid_kp_l_re  = pid_kp_l_sel && (PID_KP_L_DM_LOC  ?  ramre : iore); 
  assign pid_sp_h_re  = pid_sp_h_sel && (PID_SP_H_DM_LOC  ?  ramre : iore); 
  assign pid_sp_l_re  = pid_sp_l_sel && (PID_SP_L_DM_LOC  ?  ramre : iore); 
  assign pid_pv_h_re  = pid_pv_h_sel && (PID_PV_H_DM_LOC  ?  ramre : iore); 
  assign pid_pv_l_re  = pid_pv_l_sel && (PID_PV_L_DM_LOC  ?  ramre : iore); 
  assign pid_op_h_re  = pid_op_h_sel && (PID_OP_H_DM_LOC  ?  ramre : iore); 
  assign pid_op_l_re  = pid_op_l_sel && (PID_OP_L_DM_LOC  ?  ramre : iore); 
  // Enable register Read Data onto dbus
  assign dbus_out =  ({8{pidcr_sel}}    & pidcr_rdata)    |
                     ({8{pid_kd_h_sel}} & PID_KD_H) | 
                     ({8{pid_kd_l_sel}} & PID_KD_L) | 
                     ({8{pid_ki_h_sel}} & PID_KI_H) | 
                     ({8{pid_ki_l_sel}} & PID_KI_L) | 
                     ({8{pid_kp_h_sel}} & PID_KP_H) | 
                     ({8{pid_kp_l_sel}} & PID_KP_L) | 
                     ({8{pid_sp_h_sel}} & PID_SP_H) | 
                     ({8{pid_sp_l_sel}} & PID_SP_L) | 
                     ({8{pid_pv_h_sel}} & PID_PV_H) | 
                     ({8{pid_pv_l_sel}} & PID_PV_L) |
                     ({8{pid_op_h_sel}} & chan_output_rdata_h) |
                     ({8{pid_op_l_sel}} & chan_output_rdata_l) ;
 
  // Control of external mux during read (i.e. share read access with other XBs)
  assign io_out_en = pidcr_re    || 
                     pid_kd_h_re || 
                     pid_kd_l_re || 
                     pid_ki_h_re ||
                     pid_ki_l_re ||
                     pid_kp_h_re || 
                     pid_kp_l_re ||
                     pid_sp_h_re || 
                     pid_sp_l_re ||
                     pid_pv_h_re || 
                     pid_pv_l_re ||
                     pid_op_h_re ||
                     pid_op_l_re ;

   // Write Control Register
  assign chan_in = dbus_in[PIDCHAN_LSB +: 5];
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PIDEN   <= 1'b0;
          PIDCHAN <= 5'h0;
          pids_en <= {NUM_PIDS{1'b0}};
        end
      else if (pidcr_we)
        begin
          // Or in the enable bit from the host if the disable is not set
          PIDEN   <= dbus_in[PIDEN_BIT]  ||   (pids_en[chan_in] && ~dbus_in[PIDDIS_BIT]);
          // Select the PID to be enabled or disabled
          PIDCHAN <= chan_in;
          pids_en[chan_in] <= dbus_in[PIDEN_BIT] || (pids_en[chan_in] && ~dbus_in[PIDDIS_BIT]);
        end
      else
        begin
          PIDEN <= pids_en[PIDCHAN];
        end
    end // always @ (posedge clk or negedge rstn)

  // KD register high
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_KD_H <=8'b0;
        end
      else if (pid_kd_h_we)
        begin
          PID_KD_H <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)
  // KD register low
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_KD_L <=8'b0;
        end
      else if (pid_kd_l_we)
        begin
          PID_KD_L <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)

  // KI register high
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_KI_H <=8'b0;
        end
      else if (pid_ki_h_we)
        begin
          PID_KI_H <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)
  // KI register low
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_KI_L <=8'b0;
        end
      else if (pid_ki_l_we)
        begin
          PID_KI_L <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)

  // KP register high
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_KP_H <=8'b0;
        end
      else if (pid_kp_h_we)
        begin
          PID_KP_H <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)
  // KP register low
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_KP_L <=8'b0;
        end
      else if (pid_kp_l_we)
        begin
          PID_KP_L <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)

  // SP register high
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_SP_H <=8'b0;
        end
      else if (pid_sp_h_we)
        begin
          PID_SP_H <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)
  // SP register low
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_SP_L <=8'b0;
        end
      else if (pid_sp_l_we)
        begin
          PID_SP_L <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)

  // PV register high
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_PV_H <=8'b0;
        end
      else if (pid_pv_h_we)
        begin
          PID_PV_H <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)
  // PV register low
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          PID_PV_L <=8'b0;
        end
      else if (pid_pv_l_we)
        begin
          PID_PV_L <= dbus_in;
        end
    end // always @ (posedge clk or negedge rstn)




  // Update all user write-able variables all at once
  // This will probably change - constants, setpoints, and process variable
  // will need independent writes so user does not need to remember things
  always @(posedge clk)
    begin
      if (pidcr_we && dbus_in[PIDUP_BIT])
        begin
          chan_kd[chan_in[3:0]] <= {PID_KD_H,PID_KD_L};
          chan_ki[chan_in[3:0]] <= {PID_KI_H,PID_KI_L};
          chan_kp[chan_in[3:0]] <= {PID_KP_H,PID_KP_L};
          chan_sp[chan_in[3:0]] <= {PID_SP_H,PID_SP_L};
        end
    end // always @ (posedge clk or negedge rstn)

  // PV is writen after two write to PID_PV
  always @(posedge clk)
    begin
      if (write_pv)
        begin
          chan_pv[PIDCHAN] <= {PID_PV_H,PID_PV_L};
        end
    end // always @ (posedge clk or negedge rstn)

  // Read control reg
  assign pidcr_rdata = ({7'h0,PIDEN}   << PIDEN_BIT)  |
                       ({3'h0,PIDCHAN} << PIDCHAN_LSB);

  // Read output
  assign chan_output_rdata_l = chan_output[PIDCHAN][7:0];
  assign chan_output_rdata_h = chan_output[PIDCHAN][15:8];

  // State machine which writes internal PV after high and low byte written
  // Then triggers PID calculation
  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          state <= IDLE;
          trigger_pid <= {NUM_PIDS{1'b0}};
          write_pv <= 1'b0;
        end
      else
        begin
          case (state)
            IDLE:
              begin
                if (pid_pv_h_we || pid_pv_l_we)
                  begin
                    state <= ONE_WRITE;
                    trigger_pid <= {NUM_PIDS{1'b0}};
                    write_pv <= 1'b0;
                  end
                else
                  begin
                    state <= IDLE;
                    trigger_pid <= {NUM_PIDS{1'b0}};
                    write_pv <= 1'b0;
                  end
              end
            ONE_WRITE:
              begin
                if (pidcr_we)
                  begin
                    state <= IDLE;
                    trigger_pid <= {NUM_PIDS{1'b0}};
                    write_pv <= 1'b0;
                  end
                else if (pid_pv_h_we || pid_pv_l_we)
                  begin
                    state <= TWO_WRITE;
                    trigger_pid <= {NUM_PIDS{1'b0}};
                    write_pv <= 1'b1;
                  end
                else
                  begin
                    state <= ONE_WRITE;
                    trigger_pid <= {NUM_PIDS{1'b0}};
                    write_pv <= 1'b0;
                  end
              end
            TWO_WRITE:
              begin
                    state <= IDLE;
                    trigger_pid[PIDCHAN] <= 1'b1;
                    write_pv <= 1'b0;
              end
            default:                                          // Shouldn't get here
              begin
                    state <= IDLE;
                    trigger_pid <= {NUM_PIDS{1'b0}};
                    write_pv <= 1'b0;
              end
          endcase
        end
    end


  assign load_en = ((dbus_in[PIDUP_BIT] && pidcr_we) << chan_in);

  always @(posedge clk or negedge rstn)
    begin
      if (!rstn)
        begin
          /*AUTORESET*/
          // Beginning of autoreset for uninitialized flops
          timercnt_20 <= 15'd0;
          sample_20 <= 1'b0;
          // End of automatics
        end
      else if (en1mhz && |pids_en)
        begin
          // it takes 20000 cycles of 1MHz to get 20ms
          if (timercnt_20 == 15'd19999)
            begin
              timercnt_20 <= 15'd0;
              sample_20 <= 1'b1;
            end
          else
            begin
              timercnt_20 <= timercnt_20 + 15'd1;
            end
        end
      else
        begin
              sample_20 <= 1'b0;
        end
    end

  genvar ii;
  generate
    for (ii=0;ii<NUM_PIDS;ii++) begin : gen_chan0
      always @(posedge clk or negedge rstn) begin
        if (!rstn)
          begin
            trigger_enable[ii] <= 1'b0;
          end
        else if (sample_20)                       // if 20ms timer, tick enable PID to be triggered on next PV update
          begin
            trigger_enable[ii] <= 1'b1;
          end
        else if (trigger_pid[ii])
          begin
            trigger_enable[ii] <= 1'b0;          // Disable algorithm until next 20ms tack
          end
        else
         begin
            trigger_enable[ii] <= trigger_enable[ii];
         end
      end // always @ (posedge clk or negedge rstn)
    end // for
  endgenerate // block: gen_chan

  generate
    for (ii=0;ii<NUM_PIDS;ii++) begin : gen_chan1
      assign k1[ii] = chan_kp[ii] + chan_ki[ii] + chan_kd[ii];
      assign k2[ii] = chan_kp[ii] + (chan_kd[ii] << 1);
      assign k3[ii] = chan_kd[ii];
      assign chan_output[ii] = last_out[ii] + (k1[ii] * err[ii]) - (k2[ii] * err_1[ii]) + (k3[ii] * err_2[ii]);
    end
  endgenerate


  generate
    for (ii=0;ii<NUM_PIDS;ii++) begin : gen_chan2
      always @(posedge clk or negedge rstn) begin
        if (!rstn)
          begin
            err[ii]            <= 16'b0;
            err_1[ii]          <= 16'b0;
            err_2[ii]          <= 16'b0;
            last_out[ii]       <= 16'b0;
          end
        else if (pids_en[ii] && trigger_pid[ii] && trigger_enable[ii])
          begin
            err[ii]            <= chan_sp[ii] - chan_pv[ii];
            err_1[ii]          <= err[ii];
            err_2[ii]          <= err_1[ii];
            last_out[ii]       <= chan_output[ii];
          end
        else
          begin
            err[ii]            <= err[ii];
            err_1[ii]          <= err_1[ii];
            err_2[ii]          <= err_2[ii];
            last_out[ii]       <= last_out[ii];
          end
      end // always @ (posedge clk or negedge rstn)
    end // for
  endgenerate // block: gen_chan
  
   /////////////////////////////////
   // Assertions
   /////////////////////////////////


   /////////////////////////////////
   // Cover Points
   /////////////////////////////////

`ifdef SUP_COVER_ON
`endif

endmodule

