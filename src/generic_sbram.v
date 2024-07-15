/////////////////////////////////////////////////////////////////////////////////
// 
// Copyright (c) 2023, Jose R. Garcia (jg-fossh@protonmail.com)
// All rights reserved.
//
// The following hardware description source code is subject to the terms of the
//                  Open Hardware Description License, v. 1.0
// If a copy of the afromentioned license was not distributed with this file you
// can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt
//
/////////////////////////////////////////////////////////////////////////////////
// File name    : generic_sbram.v
// Author       : Jose R Garcia (jg-fossh@protonmail.com)
// Project Name : Generic SBRAM
// Module Name  : generic_sbram
// Description  : Infers Simple Dual Port BRAM. Most modern synthesis tools can
//  infer this code as a memory structure. The P_SBRAM_MASK_MSB is used to
//  determine if the RAM writes are bit or byte maskable. For now write mask
//  set P_SBRAM_MASK_MSB to 0.
//
// Additional Comments:
//
//  ** These get inferred properly most of the time(depending on the tool version) **
//
//   1. Lattice iCE40 sysMEM (RAM4K)
//      | Config | ADDR | DATA | MASK
//      | 256x16 | 7:0  | 15:0 | 15:0
//      | 512x8  | 8:0  | 7:0  | N/A
//      | 1024x4 | 9:0  | 3:0  | N/A
//      | 2048x2 | 10:0 | 1:0  | N/A
//
//   2. Anlogic Eagle BRAM9K
//      | Config | ADDR | DATA | MASK
//      | 512x16 | 8:0  | 15:0 | [3:0]
//      | 512x18 | 8:0  | 17:0 | [3:0]
//      | 1024x8 | 9:0  | 7:0  | [3:0]/[1:0]
//      | 1024x9 | 9:0  | 8:0  | [3:0]/[1:0]
//      | 2048x4 | 10:0 | 3:0  | N/A
//      | 4096x2 | 11:0 | 1:0  | N/A
//      | 8192x1 | 12:0 | 0    | N/A
//
//   3. Anlogic Eagle BRAM32K
//      | Config  | ADDR | DATA | MASK
//      | 2048x16 | 10:0 | 15:0 | N/A
//      | 4096x8  | 11:0 | 7:0  | LSBs of ADDR (*not working*)
//
// For other technologies check specifics in their data sheet.
/////////////////////////////////////////////////////////////////////////////////
module generic_sbram #(
  // Compile time configurable parameters
  parameter integer P_SBRAM_DATA_MSB  = 15,
  parameter integer P_SBRAM_ADDR_MSB  =  6,
  parameter integer P_SBRAM_MASK_MSB  =  0,
  parameter integer P_SBRAM_HAS_FILE  =  0,
  parameter         P_SBRAM_INIT_FILE =  0
)(
  input                       i_ce,
  input                       i_wclk,
  input                       i_rclk,
  input  [P_SBRAM_ADDR_MSB:0] i_waddr,
  input  [P_SBRAM_ADDR_MSB:0] i_raddr,
  input                       i_we,
  input  [P_SBRAM_DATA_MSB:0] i_mask, // 0=writes, 1=masks
  input  [P_SBRAM_DATA_MSB:0] i_wdata,
  output [P_SBRAM_DATA_MSB:0] o_rdata
);

  ///////////////////////////////////////////////////////////////////////////////
  // Functions Declaration
  ///////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////
  // Function    : MASK MODE DETECT
  // Description : Detects the mask mode base on the mask MSB and data MSB.
  //   0=No mask; 1=Mask Bit; 2=Mask Byte.
  ///////////////////////////////////////////////////////////////////////////////
  function automatic integer F_MASK_MODE_DETECT (
    input integer data_msb,
    input integer mask_msb
  );
    begin
      //
      F_MASK_MODE_DETECT = 0;
      if (mask_msb == data_msb) begin
        F_MASK_MODE_DETECT = 1;
      end
      else if (mask_msb == ((data_msb+1)/8)-1) begin
        F_MASK_MODE_DETECT = 2;
      end
      else begin
        F_MASK_MODE_DETECT = 0;
      end
    end
  endfunction // F_MASK_MODE_DETECT

  ///////////////////////////////////////////////////////////////////////////////
  // Internal Parameter Declarations
  ///////////////////////////////////////////////////////////////////////////////
  localparam integer L_RAM_DEPTH = 1<<(P_SBRAM_ADDR_MSB+1);
  localparam integer L_MASK_MODE = F_MASK_MODE_DETECT(P_SBRAM_DATA_MSB, P_SBRAM_MASK_MSB);

  /////////////////////////////////////////////////////////////////////////////
  // Internal Parameter Declarations
  /////////////////////////////////////////////////////////////////////////////
  // Signals Definition
  reg [P_SBRAM_DATA_MSB:0] r_rdata;

  ///////////////////////////////////////////////////////////////////////////////
  //            ********      Architecture Declaration      ********           //
  ///////////////////////////////////////////////////////////////////////////////

  generate
    if (L_MASK_MODE == 0) begin : Write_Mask_Mode_Disable

      /* verilator lint_off MULTIDRIVEN */
      reg [P_SBRAM_DATA_MSB:0] r_ram [0:L_RAM_DEPTH-1] /*verilator public*/;
      /* verilator lint_on MULTIDRIVEN */

      if (P_SBRAM_HAS_FILE == 1) begin : Generate_Init_File_Read
        initial begin
          // Load initial states of the brams from a file.
          $readmemb(P_SBRAM_INIT_FILE, r_ram);
        end
      end // Generate_Init_File_Read

      ///////////////////////////////////////////////////////////////////////////////
      // Process     : RAM Write Access
      // Description : Synchronous writes to memory.
      ///////////////////////////////////////////////////////////////////////////////
      always @(posedge i_wclk) begin : RAM_Write_Port_Access
        if (i_ce == 1'b1 && i_we == 1'b1) begin
          // RAM Write Port Access
          r_ram[i_waddr] <= i_wdata;
        end
      end // RAM_Write_Port_Access

      ///////////////////////////////////////////////////////////////////////////////
      // Process     : RAM Read Port Access
      // Description : Synchronous reads memory. There is no read enable signal for
      //               the read side, hence it is always reading on every clock
      //               transition.
      ///////////////////////////////////////////////////////////////////////////////
      always @(posedge i_rclk) begin : RAM_Read_Port_Access
        if (i_ce == 1'b1) begin
          r_rdata <= r_ram[i_raddr];
        end
      end // RAM_Read_Port_Access
      assign o_rdata = r_rdata;

    end // Write_Mask_Mode_Disable
  endgenerate

  generate
    if (L_MASK_MODE == 1) begin : Write_Mask_Mode_Bit

      /* verilator lint_off MULTIDRIVEN */
      reg [P_SBRAM_DATA_MSB:0] r_ram [0:L_RAM_DEPTH-1] /*verilator public*/;
      /* verilator lint_on MULTIDRIVEN */

      if (P_SBRAM_HAS_FILE == 1) begin : Generate_Init_File_Read
        initial begin
          // Load initial states of the brams from a file.
          $readmemb(P_SBRAM_INIT_FILE, r_ram);
        end
      end // Generate_Init_File_Read

      //
      integer write_index;

      ///////////////////////////////////////////////////////////////////////////////
      // Process     : RAM Write Port Access
      // Description : Synchronous writes to ram with bit masking.
      ///////////////////////////////////////////////////////////////////////////////
      always @(posedge i_wclk) begin : RAM_Write_Port_Access
        if (i_ce == 1'b1 && i_we == 1'b1) begin
          for (write_index = 0; write_index <= P_SBRAM_DATA_MSB; write_index = write_index+1) begin
            if (i_mask[write_index] == 1'b0) begin
              r_ram[i_waddr][write_index] <= i_wdata[write_index];
            end
          end
        end
      end // RAM_Write_Port_Access

      ///////////////////////////////////////////////////////////////////////////////
      // Process     : RAM Read Port Access
      // Description : Synchronous reads memory. There is no read enable signal for
      //               the read side, hence it is always reading on every clock
      //               transition.
      ///////////////////////////////////////////////////////////////////////////////
      always @(posedge i_rclk) begin : RAM_Read_Port_Access
        if (i_ce == 1'b1) begin
          r_rdata <= r_ram[i_raddr];
        end
      end // RAM_Read_Port_Access
      assign o_rdata = r_rdata;

    end // Write_Mask_Mode_Bit
  endgenerate

  generate
    if (L_MASK_MODE == 2) begin : Write_Mask_Mode_Byte

      /* verilator lint_off MULTIDRIVEN */
      reg [P_SBRAM_DATA_MSB:0] r_ram [0:L_RAM_DEPTH-1] /*verilator public*/;
      /* verilator lint_on MULTIDRIVEN */

      if (P_SBRAM_HAS_FILE == 1) begin : Generate_Init_File_Read
        initial begin
          // Load initial states of the brams from a file.
          $readmemb(P_SBRAM_INIT_FILE, r_ram);
        end
      end // Generate_Init_File_Read

      //
      integer write_index;
      integer read_index;

      ///////////////////////////////////////////////////////////////////////////////
      // Process     : RAM Write Port Access
      // Description : Synchronous writes to ram with bit masking.
      ///////////////////////////////////////////////////////////////////////////////
      always @(posedge i_wclk) begin : RAM_Write_Port_Access
        if (i_ce == 1'b1 && i_we == 1'b1) begin
          for (write_index = 0; write_index <= P_SBRAM_MASK_MSB; write_index = write_index+1) begin
            if (i_mask[write_index] == 1'b0) begin
              r_ram[i_waddr][write_index*8 +: 8] <= i_wdata[write_index*8 +: 8];
            end
          end
        end
      end // RAM_Write_Port_Access

      ///////////////////////////////////////////////////////////////////////////////
      // Process     : RAM Read Port Access
      // Description : Synchronous reads memory. There is no read enable signal for
      //               the read side, hence it is always reading on every clock
      //               transition.
      ///////////////////////////////////////////////////////////////////////////////
      always @(posedge i_rclk) begin : RAM_Read_Port_Access
        if (i_ce == 1'b1) begin
          for (read_index = 0; read_index <= P_SBRAM_MASK_MSB; read_index = read_index+1) begin
            r_rdata <= r_ram[i_raddr]; // RAM Read Port 0 Access
          end
        end
      end // RAM_Read_Port_Access
      assign o_rdata = r_rdata;

    end // Write_Mask_Mode_Byte
  endgenerate

endmodule // generic_sbram
