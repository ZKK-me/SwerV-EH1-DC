//********************************************************************************
// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or it's affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//********************************************************************************
////////////////////////////////////////////////////
//   DCACHE DATA & TAG MODULE WRAPPER              //
/////////////////////////////////////////////////////
module lsu_dc_mem
  (
      input logic free_clk,
      input logic clk,
      input logic rst_l,
      input logic clk_override,
      input logic dec_tlu_core_ecc_disable,

      input logic [31:2]  dc_rw_addr,
      input logic [3:0]   dc_wr_en  ,  // Which way to write
      input logic         dc_rd_en  ,  // Read enable

      input logic [15:2]               dc_debug_addr,      // Read/Write addresss to the Dcache.
      input logic                      dc_debug_rd_en,     // Dcache debug rd
      input logic                      dc_debug_wr_en,     // Dcache debug wr
      input logic                      dc_debug_tag_array, // Debug tag array
      input logic [3:0]                dc_debug_way,       // Debug way. Rd or Wr.
      input logic [127:0]              dc_premux_data,     // Premux data to be muxed with each way of the Dcache.
      input logic                      dc_sel_premux_data, // Select the pre_muxed data



`ifdef RV_DCACHE_ECC
      input  logic [83:0]               dc_wr_data,         // Data to fill to the Dcache. With ECC
      output logic [167:0]              dc_rd_data ,        // Data read from Dcache. 2x64bits + parity bits. F2 stage. With ECC
      output logic [24:0]               dctag_debug_rd_data,// Debug dcache tag.
      input logic  [41:0]               dc_debug_wr_data,   // Debug wr cache.
`else
      input  logic [67:0]               dc_wr_data,         // Data to fill to the Dcache. With Parity
      output logic [135:0]              dc_rd_data ,        // Data read from Dcache. 2x64bits + parity bits. F2 stage. With Parity
      output logic [20:0]               dctag_debug_rd_data,// Debug dcache tag.
      input logic  [33:0]               dc_debug_wr_data,   // Debug wr cache.
`endif


      input logic [3:0]   dc_tag_valid,  // Valid from the D$ tag valid outside (in flops).

      output logic [3:0]   dc_rd_hit,   // dc_rd_hit[3:0]
      output logic         dc_tag_perr, // Tag Parity error
      input  logic         scan_mode
      ) ;

`include "global.h"

   DC_TAG #( .DCACHE_TAG_HIGH(DCACHE_TAG_HIGH) ,
             .DCACHE_TAG_LOW(DCACHE_TAG_LOW) ,
             .DCACHE_TAG_DEPTH(DCACHE_TAG_DEPTH)
             ) dc_tag_inst
          (
           .*,
           .dc_wr_en     (dc_wr_en[3:0]),
           .dc_debug_addr(dc_debug_addr[DCACHE_TAG_HIGH-1:2]),
           .dc_rw_addr   (dc_rw_addr[31:3])
           ) ;

   DC_DATA #( .DCACHE_TAG_HIGH(DCACHE_TAG_HIGH) ,
              .DCACHE_TAG_LOW(DCACHE_TAG_LOW) ,
              .DCACHE_DC_DEPTH(DCACHE_DC_DEPTH)
             ) dc_data_inst
          (
           .*,
           .dc_wr_en     (dc_wr_en[3:0]),
           .dc_debug_addr(dc_debug_addr[DCACHE_TAG_HIGH-1:2]),
           .dc_rw_addr   (dc_rw_addr[DCACHE_TAG_HIGH-1:2])
           ) ;

 endmodule


/////////////////////////////////////////////////
////// DCACHE DATA MODULE    ////////////////////
/////////////////////////////////////////////////
module DC_DATA #(parameter DCACHE_TAG_HIGH = 16 ,
                           DCACHE_TAG_LOW=6 ,
                           DCACHE_DC_DEPTH=1024
                                        )
     (
      input logic free_clk,
      input logic clk,
      input logic rst_l,
      input logic clk_override,

      input logic [DCACHE_TAG_HIGH-1:2]  dc_rw_addr,
      input logic [3:0]                  dc_wr_en,
      input logic                        dc_rd_en,  // Read enable
`ifdef RV_DCACHE_ECC
      input  logic [83:0]               dc_wr_data,         // Data to fill to the Dcache. With ECC
      output logic [167:0]              dc_rd_data ,        // Data read from Dcache. 2x64bits + parity bits. F2 stage. With ECC
      input  logic [41:0]               dc_debug_wr_data,   // Debug wr cache.
`else
      input  logic [67:0]               dc_wr_data,         // Data to fill to the Dcache. With Parity
      output logic [135:0]              dc_rd_data ,        // Data read from Dcache. 2x64bits + parity bits. F2 stage. With Parity
      input  logic [33:0]               dc_debug_wr_data,   // Debug wr cache.
`endif


      input logic [DCACHE_TAG_HIGH-1:2]  dc_debug_addr,      // Read/Write addresss to the Dcache.
      input logic                        dc_debug_rd_en,     // Dcache debug rd
      input logic                        dc_debug_wr_en,     // Dcache debug wr
      input logic                        dc_debug_tag_array, // Debug tag array
      input logic [3:0]                  dc_debug_way,       // Debug way. Rd or Wr.
      input logic [127:0]                dc_premux_data,     // Premux data to be muxed with each way of the Dcache.
      input logic                        dc_sel_premux_data, // Select the pre_muxed data

      input logic [3:0]          dc_rd_hit,

      input  logic               scan_mode

      ) ;

   logic [5:4]             dc_rw_addr_ff;


   logic [3:0][3:0]       dc_b_sb_wren, dc_bank_way_clken, dc_bank_way_clk;    // way, bank

   logic                   dc_debug_sel_sb0 ;
   logic                   dc_debug_sel_sb1 ;
   logic                   dc_debug_sel_sb2 ;
   logic                   dc_debug_sel_sb3 ;


`ifdef RV_DCACHE_ECC
   logic [3:0] [167:0]     bank_set_dout;
   logic [3:0][167:0]      wb_dout ; //
   logic [3:0][41:0]       dc_sb_wr_data;
`else
   logic [3:0] [135:0]     bank_set_dout;
   logic [3:0] [135:0]     wb_dout ; // bank , way , size
   logic [3:0] [33:0]      dc_sb_wr_data;
`endif

   logic                   dc_b_rden;
   logic [3:0]             dc_debug_rd_way_en;   // debug wr_way
   logic [3:0]             dc_debug_rd_way_en_ff;   // debug wr_way
   logic [3:0]             dc_debug_wr_way_en;   // debug wr_way
   logic [DCACHE_TAG_HIGH-1:4]  dc_rw_addr_q;

   assign  dc_debug_rd_way_en[3:0] =  {4{dc_debug_rd_en & ~dc_debug_tag_array}} & dc_debug_way[3:0] ;
   assign  dc_debug_wr_way_en[3:0] =  {4{dc_debug_wr_en & ~dc_debug_tag_array}} & dc_debug_way[3:0] ;

   assign  dc_b_sb_wren[0][3:0]  = (dc_wr_en[3:0]           & {4{~dc_rw_addr[3]}} ) |
                                   (dc_debug_wr_way_en[3:0] & {4{dc_debug_addr[3:2] == 2'b00}}) ;
   assign  dc_b_sb_wren[1][3:0]  = (dc_wr_en[3:0]           & {4{~dc_rw_addr[3]}} ) |
                                   (dc_debug_wr_way_en[3:0] & {4{dc_debug_addr[3:2] == 2'b01}}) ;
   assign  dc_b_sb_wren[2][3:0]  = (dc_wr_en[3:0]           & {4{dc_rw_addr[3]}} ) |
                                   (dc_debug_wr_way_en[3:0] & {4{dc_debug_addr[3:2] == 2'b10}}) ;
   assign  dc_b_sb_wren[3][3:0]  = (dc_wr_en[3:0]           & {4{dc_rw_addr[3]}} ) |
                                   (dc_debug_wr_way_en[3:0] & {4{dc_debug_addr[3:2] == 2'b11}}) ;

   assign  dc_debug_sel_sb0       =  (dc_debug_addr[3:2] == 2'b00 ) ;
   assign  dc_debug_sel_sb1       =  (dc_debug_addr[3:2] == 2'b01 ) ;
   assign  dc_debug_sel_sb2       =  (dc_debug_addr[3:2] == 2'b10 ) ;
   assign  dc_debug_sel_sb3       =  (dc_debug_addr[3:2] == 2'b11 ) ;

`ifdef RV_DCACHE_ECC

   assign  dc_sb_wr_data[0][41:0]   =  (dc_debug_sel_sb0 & dc_debug_wr_en) ? {dc_debug_wr_data[41:0]} :
                                                                             dc_wr_data[41:0] ;
   assign  dc_sb_wr_data[1][41:0]   =  (dc_debug_sel_sb1 & dc_debug_wr_en) ? {dc_debug_wr_data[41:0]} :
                                                                             dc_wr_data[83:42] ;
   assign  dc_sb_wr_data[2][41:0]   =  (dc_debug_sel_sb2 & dc_debug_wr_en) ? {dc_debug_wr_data[41:0]} :
                                                                             dc_wr_data[41:0] ;
   assign  dc_sb_wr_data[3][41:0]   =  (dc_debug_sel_sb3 & dc_debug_wr_en) ? {dc_debug_wr_data[41:0]} :
                                                                             dc_wr_data[83:42] ;
`else
   assign  dc_sb_wr_data[0][33:0]   =  (dc_debug_sel_sb0 & dc_debug_wr_en) ? dc_debug_wr_data[33:0] :
                                                                             dc_wr_data[33:0] ;
   assign  dc_sb_wr_data[1][33:0]   =  (dc_debug_sel_sb1 & dc_debug_wr_en) ? dc_debug_wr_data[33:0] :
                                                                             dc_wr_data[67:34] ;
   assign  dc_sb_wr_data[2][33:0]   =  (dc_debug_sel_sb2 & dc_debug_wr_en) ? dc_debug_wr_data[33:0] :
                                                                             dc_wr_data[33:0] ;
   assign  dc_sb_wr_data[3][33:0]   =  (dc_debug_sel_sb3 & dc_debug_wr_en) ? dc_debug_wr_data[33:0] :
                                                                             dc_wr_data[67:34] ;
`endif


// bank read enables

   assign  dc_b_rden       = (dc_rd_en   | dc_debug_rd_en );

   logic [3:0] dc_bank_read;
   logic [3:0] dc_bank_read_ff;
   assign dc_bank_read[0] = (dc_b_rden) & (~|dc_rw_addr[3:2] | dc_debug_rd_en);
   assign dc_bank_read[1] = (dc_b_rden) & (~dc_rw_addr[3] | dc_debug_rd_en);
   assign dc_bank_read[2] = (dc_b_rden) & (~&dc_rw_addr[3:2] | dc_debug_rd_en);
   assign dc_bank_read[3] = (dc_b_rden) | dc_debug_rd_en;

    assign dc_rw_addr_q[DCACHE_TAG_HIGH-1:4] = (dc_debug_rd_en | dc_debug_wr_en) ?
                                                dc_debug_addr[DCACHE_TAG_HIGH-1:4] :
                                                dc_rw_addr[DCACHE_TAG_HIGH-1:4] ;

   logic dc_debug_rd_en_ff;

   rvdff #(2) adr_ff (.*,
                    .clk(free_clk),
                    .din ({dc_rw_addr_q[5:4]}),
                    .dout({dc_rw_addr_ff[5:4]}));

   rvdff #(4) bank_adr_ff (.*,
                    .clk(free_clk),
                    .din (dc_bank_read[3:0]),
                    .dout(dc_bank_read_ff[3:0]));

   rvdff #(5) debug_rd_wy_ff (.*,
                    .clk(free_clk),
                    .din ({dc_debug_rd_way_en[3:0], dc_debug_rd_en}),
                    .dout({dc_debug_rd_way_en_ff[3:0], dc_debug_rd_en_ff}));

localparam NUM_WAYS=4 ;
localparam NUM_SUBBANKS=4 ;


     for (genvar i=0; i<NUM_WAYS; i++) begin: WAYS


        for (genvar k=0; k<NUM_SUBBANKS; k++) begin: SUBBANKS   // 16B subbank

           // way3-bank3, way3-bank2, ... way0-bank0
           assign  dc_bank_way_clken[i][k]   = dc_bank_read[k] |  dc_b_sb_wren[k][i];

           rvoclkhdr bank_way_bank_c1_cgc  ( .en(dc_bank_way_clken[i][k] | clk_override), .l1clk(dc_bank_way_clk[i][k]), .* );

        `ifdef RV_DCACHE_ECC
         `RV_DCACHE_DATA_CELL  dc_bank_sb_way_data (
                                     .CLK(dc_bank_way_clk[i][k]),
                                     .WE (dc_b_sb_wren[k][i]),
                                     .D  (dc_sb_wr_data[k][41:0]),
                                     .ADR(dc_rw_addr_q[DCACHE_TAG_HIGH-1:4]),
                                     .Q  (wb_dout[i][(k+1)*42-1:k*42])
                                    );
        `else
         `RV_DCACHE_DATA_CELL  dc_bank_sb_way_data (
                                     .CLK(dc_bank_way_clk[i][k]),
                                     .WE (dc_b_sb_wren[k][i]),
                                     .D  (dc_sb_wr_data[k][33:0]),
                                     .ADR(dc_rw_addr_q[DCACHE_TAG_HIGH-1:4]),
                                     .Q  (wb_dout[i][(k+1)*34-1:k*34])
                                    );
        `endif
        end // block: SUBBANKS

      end


   logic [3:0] dc_rd_hit_q;
   assign dc_rd_hit_q[3:0] = dc_debug_rd_en_ff ? dc_debug_rd_way_en_ff[3:0] : dc_rd_hit[3:0] ;

   // set mux
`ifdef RV_DCACHE_ECC
   logic [167:0] dc_premux_data_ext;
   logic [3:0] [167:0] wb_dout_way;
   logic [3:0] [167:0] wb_dout_way_with_premux;

   assign dc_premux_data_ext[167:0] =  {10'b0,dc_premux_data[127:96],10'b0,dc_premux_data[95:64] ,10'b0,dc_premux_data[63:32],10'b0,dc_premux_data[31:0]};
   assign wb_dout_way[0][167:0]       = wb_dout[0][167:0] & {  {42{dc_bank_read_ff[3]}} ,  {42{dc_bank_read_ff[2]}}  ,  {42{dc_bank_read_ff[1]}}  ,  {42{dc_bank_read_ff[0]}} };
   assign wb_dout_way[1][167:0]       = wb_dout[1][167:0] & {  {42{dc_bank_read_ff[3]}} ,  {42{dc_bank_read_ff[2]}}  ,  {42{dc_bank_read_ff[1]}}  ,  {42{dc_bank_read_ff[0]}} };
   assign wb_dout_way[2][167:0]       = wb_dout[2][167:0] & {  {42{dc_bank_read_ff[3]}} ,  {42{dc_bank_read_ff[2]}}  ,  {42{dc_bank_read_ff[1]}}  ,  {42{dc_bank_read_ff[0]}} };
   assign wb_dout_way[3][167:0]       = wb_dout[3][167:0] & {  {42{dc_bank_read_ff[3]}} ,  {42{dc_bank_read_ff[2]}}  ,  {42{dc_bank_read_ff[1]}}  ,  {42{dc_bank_read_ff[0]}} };


   assign wb_dout_way_with_premux[0][167:0]  =  dc_sel_premux_data ? dc_premux_data_ext[167:0] : wb_dout_way[0][167:0] ;
   assign wb_dout_way_with_premux[1][167:0]  =  dc_sel_premux_data ? dc_premux_data_ext[167:0] : wb_dout_way[1][167:0] ;
   assign wb_dout_way_with_premux[2][167:0]  =  dc_sel_premux_data ? dc_premux_data_ext[167:0] : wb_dout_way[2][167:0] ;
   assign wb_dout_way_with_premux[3][167:0]  =  dc_sel_premux_data ? dc_premux_data_ext[167:0] : wb_dout_way[3][167:0] ;

   assign dc_rd_data[167:0]       = ({168{dc_rd_hit_q[0] | dc_sel_premux_data}} &  wb_dout_way_with_premux[0][167:0]) |
                                    ({168{dc_rd_hit_q[1] | dc_sel_premux_data}} &  wb_dout_way_with_premux[1][167:0]) |
                                    ({168{dc_rd_hit_q[2] | dc_sel_premux_data}} &  wb_dout_way_with_premux[2][167:0]) |
                                    ({168{dc_rd_hit_q[3] | dc_sel_premux_data}} &  wb_dout_way_with_premux[3][167:0]) ;

`else
   logic       [135:0] dc_premux_data_ext;
   logic [3:0] [135:0] wb_dout_way;
   logic [3:0] [135:0] wb_dout_way_with_premux;

   assign dc_premux_data_ext[135:0]   = {2'b0,dc_premux_data[127:96],2'b0,dc_premux_data[95:64] ,2'b0,dc_premux_data[63:32],2'b0,dc_premux_data[31:0]};
   assign wb_dout_way[0][135:0]       = wb_dout[0][135:0] &  {  {34{dc_bank_read_ff[3]}} ,  {34{dc_bank_read_ff[2]}}  ,  {34{dc_bank_read_ff[1]}}  ,  {34{dc_bank_read_ff[0]}} };
   assign wb_dout_way[1][135:0]       = wb_dout[1][135:0] &  {  {34{dc_bank_read_ff[3]}} ,  {34{dc_bank_read_ff[2]}}  ,  {34{dc_bank_read_ff[1]}}  ,  {34{dc_bank_read_ff[0]}} };
   assign wb_dout_way[2][135:0]       = wb_dout[2][135:0] &  {  {34{dc_bank_read_ff[3]}} ,  {34{dc_bank_read_ff[2]}}  ,  {34{dc_bank_read_ff[1]}}  ,  {34{dc_bank_read_ff[0]}} };
   assign wb_dout_way[3][135:0]       = wb_dout[3][135:0] &  {  {34{dc_bank_read_ff[3]}} ,  {34{dc_bank_read_ff[2]}}  ,  {34{dc_bank_read_ff[1]}}  ,  {34{dc_bank_read_ff[0]}} };

   assign wb_dout_way_with_premux[0][135:0]  =  dc_sel_premux_data ? dc_premux_data_ext[135:0] : wb_dout_way[0][135:0] ;
   assign wb_dout_way_with_premux[1][135:0]  =  dc_sel_premux_data ? dc_premux_data_ext[135:0] : wb_dout_way[1][135:0] ;
   assign wb_dout_way_with_premux[2][135:0]  =  dc_sel_premux_data ? dc_premux_data_ext[135:0] : wb_dout_way[2][135:0] ;
   assign wb_dout_way_with_premux[3][135:0]  =  dc_sel_premux_data ? dc_premux_data_ext[135:0] : wb_dout_way[3][135:0] ;

   assign dc_rd_data[135:0]       = ({136{dc_rd_hit_q[0] | dc_sel_premux_data}} &  wb_dout_way_with_premux[0][135:0]) |
                                    ({136{dc_rd_hit_q[1] | dc_sel_premux_data}} &  wb_dout_way_with_premux[1][135:0]) |
                                    ({136{dc_rd_hit_q[2] | dc_sel_premux_data}} &  wb_dout_way_with_premux[2][135:0]) |
                                    ({136{dc_rd_hit_q[3] | dc_sel_premux_data}} &  wb_dout_way_with_premux[3][135:0]) ;

`endif

 endmodule


/////////////////////////////////////////////////
////// DCACHE TAG MODULE     ////////////////////
/////////////////////////////////////////////////
module DC_TAG #(parameter DCACHE_TAG_HIGH = 16 ,
                          DCACHE_TAG_LOW=6 ,
                          DCACHE_TAG_DEPTH=1024
                                        )
     (
      input logic free_clk,
      input logic clk,
      input logic rst_l,
      input logic clk_override,
      input logic dec_tlu_core_ecc_disable,

      input logic [31:3]  dc_rw_addr,

      input logic [3:0]   dc_wr_en,  // way
      input logic [3:0]   dc_tag_valid,
      input logic         dc_rd_en,

      input logic [DCACHE_TAG_HIGH-1:2]  dc_debug_addr,      // Read/Write addresss to the Dcache.
      input logic                        dc_debug_rd_en,     // Dcache debug rd
      input logic                        dc_debug_wr_en,     // Dcache debug wr
      input logic                        dc_debug_tag_array, // Debug tag array
      input logic [3:0]                  dc_debug_way,       // Debug way. Rd or Wr.




`ifdef RV_DCACHE_ECC
      output logic [24:0]  dctag_debug_rd_data,
      input  logic [41:0]  dc_debug_wr_data,   // Debug wr cache.
`else
      output logic [20:0]  dctag_debug_rd_data,
      input  logic [33:0]  dc_debug_wr_data,   // Debug wr cache.
`endif
      output logic [3:0]   dc_rd_hit,
      output logic         dc_tag_perr,
      input  logic         scan_mode

      ) ;

`ifdef RV_DCACHE_ECC
   logic [3:0] [24:0] dc_tag_data_raw;
   logic [3:0] [37:DCACHE_TAG_HIGH] w_tout;
   logic [24:0] dc_tag_wr_data ;
   logic [3:0] [31:0] dc_tag_corrected_data_unc;
   logic [3:0] [06:0] dc_tag_corrected_ecc_unc;
   logic [3:0]        dc_tag_single_ecc_error;
   logic [3:0]        dc_tag_double_ecc_error;
`else
   logic [3:0] [20:0] dc_tag_data_raw;
   logic [3:0] [32:DCACHE_TAG_HIGH] w_tout;
   logic [20:0] dc_tag_wr_data ;
`endif

   logic [3:0]  dc_tag_way_perr ;
   logic [3:0]  dc_debug_rd_way_en ;
   logic [3:0]  dc_debug_rd_way_en_ff ;

   logic [DCACHE_TAG_HIGH-1:6]  dc_rw_addr_q;
   logic [31:4]         dc_rw_addr_ff;
   logic [3:0]          dc_tag_wren   ; // way
   logic [3:0]          dc_tag_wren_q   ; // way
   logic [3:0]          dc_tag_clk ;
   logic [3:0]          dc_tag_clken ;
   logic [3:0]          dc_debug_wr_way_en;   // debug wr_way

   assign  dc_tag_wren [3:0]  = dc_wr_en[3:0] & {4{dc_rw_addr[5:3] == 3'b111}} ;
   assign  dc_tag_clken[3:0]  = {4{dc_rd_en | clk_override}} | dc_wr_en[3:0] | dc_debug_wr_way_en[3:0] | dc_debug_rd_way_en[3:0];

   rvdff #(32-DCACHE_TAG_HIGH) adr_ff (.*,
                    .clk(free_clk),
                    .din ({dc_rw_addr[31:DCACHE_TAG_HIGH]}),
                    .dout({dc_rw_addr_ff[31:DCACHE_TAG_HIGH]}));


   localparam TOP_BITS = 21+DCACHE_TAG_HIGH-33 ;
   localparam NUM_WAYS=4 ;
   // tags



   assign  dc_debug_rd_way_en[3:0] =  {4{dc_debug_rd_en & dc_debug_tag_array}} & dc_debug_way[3:0] ;
   assign  dc_debug_wr_way_en[3:0] =  {4{dc_debug_wr_en & dc_debug_tag_array}} & dc_debug_way[3:0] ;

   assign  dc_tag_wren_q[3:0]  =  dc_tag_wren[3:0]          |
                                  dc_debug_wr_way_en[3:0]   ;

if (DCACHE_TAG_HIGH == 12) begin: SMALLEST
 `ifdef RV_DCACHE_ECC
     logic [6:0] dc_tag_ecc;
           rvecc_encode  tag_ecc_encode (
                                  .din    ({{DCACHE_TAG_HIGH{1'b0}}, dc_rw_addr[31:DCACHE_TAG_HIGH]}),
                                  .ecc_out({ dc_tag_ecc[6:0]}));

   assign  dc_tag_wr_data[24:0] = (dc_debug_wr_en & dc_debug_tag_array) ?
                                  {dc_debug_wr_data[36:32], dc_debug_wr_data[31:12]} :
                                  {dc_tag_ecc[4:0], dc_rw_addr[31:DCACHE_TAG_HIGH]} ;
 `else
   logic   dc_tag_parity ;
           rveven_paritygen #(32-DCACHE_TAG_HIGH) pargen  (.data_in   (dc_rw_addr[31:DCACHE_TAG_HIGH]),
                                                 .parity_out(dc_tag_parity));

   assign  dc_tag_wr_data[20:0] = (dc_debug_wr_en & dc_debug_tag_array) ?
                                  {dc_debug_wr_data[32], dc_debug_wr_data[31:12]} :
                                  {dc_tag_parity, dc_rw_addr[31:DCACHE_TAG_HIGH]} ;
 `endif
end else begin: OTHERS
 `ifdef RV_DCACHE_ECC
   logic [6:0] dc_tag_ecc;
           rvecc_encode  tag_ecc_encode (
                                  .din    ({{DCACHE_TAG_HIGH{1'b0}}, dc_rw_addr[31:DCACHE_TAG_HIGH]}),
                                  .ecc_out({ dc_tag_ecc[6:0]}));

   assign  dc_tag_wr_data[24:0] = (dc_debug_wr_en & dc_debug_tag_array) ?
                                  {dc_debug_wr_data[36:32], dc_debug_wr_data[31:12]} :
                                  {dc_tag_ecc[4:0], {TOP_BITS{1'b0}},dc_rw_addr[31:DCACHE_TAG_HIGH]} ;

 `else
   logic   dc_tag_parity ;
           rveven_paritygen #(32-DCACHE_TAG_HIGH) pargen  (.data_in   (dc_rw_addr[31:DCACHE_TAG_HIGH]),
                                                 .parity_out(dc_tag_parity));
   assign  dc_tag_wr_data[20:0] = (dc_debug_wr_en & dc_debug_tag_array) ?
                                  {dc_debug_wr_data[32], dc_debug_wr_data[31:12]} :
                                  {dc_tag_parity, {TOP_BITS{1'b0}},dc_rw_addr[31:DCACHE_TAG_HIGH]} ;
 `endif
end

    assign dc_rw_addr_q[DCACHE_TAG_HIGH-1:6] = (dc_debug_rd_en | dc_debug_wr_en) ?
                                                dc_debug_addr[DCACHE_TAG_HIGH-1:6] :
                                                dc_rw_addr[DCACHE_TAG_HIGH-1:6] ;


   rvdff #(4) tag_rd_wy_ff (.*,
                    .clk(free_clk),
                    .din ({dc_debug_rd_way_en[3:0]}),
                    .dout({dc_debug_rd_way_en_ff[3:0]}));




   for (genvar i=0; i<NUM_WAYS; i++) begin: WAYS

      rvoclkhdr dc_tag_c1_cgc  ( .en(dc_tag_clken[i]), .l1clk(dc_tag_clk[i]), .* );

     if (DCACHE_TAG_DEPTH == 64 ) begin : DCACHE_SZ_16
      `ifdef RV_DCACHE_ECC
         ram_64x25  dc_way_tag (
                                     .CLK(dc_tag_clk[i]),
                                     .WE (dc_tag_wren_q[i]),
                                     .D  (dc_tag_wr_data[24:0]),
                                     .ADR(dc_rw_addr_q[DCACHE_TAG_HIGH-1:DCACHE_TAG_LOW]),
                                     .Q  (dc_tag_data_raw[i][24:0])
                                    );


         assign w_tout[i][31:DCACHE_TAG_HIGH] = dc_tag_data_raw[i][31-DCACHE_TAG_HIGH:0] ;
         assign w_tout[i][36:32]              = dc_tag_data_raw[i][24:20] ;

         rvecc_decode  ecc_decode (
                           .en(~dec_tlu_core_ecc_disable),
                           .sed_ded ( 1'b1 ),    // 1 : means only detection
                           .din({12'b0,dc_tag_data_raw[i][19:0]}),
                           .ecc_in({2'b0, dc_tag_data_raw[i][24:20]}),
                           .dout(dc_tag_corrected_data_unc[i][31:0]),
                           .ecc_out(dc_tag_corrected_ecc_unc[i][6:0]),
                           .single_ecc_error(dc_tag_single_ecc_error[i]),
                           .double_ecc_error(dc_tag_double_ecc_error[i]));

          assign dc_tag_way_perr[i]= dc_tag_single_ecc_error[i] | dc_tag_double_ecc_error[i]  ;
      `else
         ram_64x21  dc_way_tag (
                                     .CLK(dc_tag_clk[i]),
                                     .WE (dc_tag_wren_q[i]),
                                     .D  (dc_tag_wr_data[20:0]),
                                     .ADR(dc_rw_addr_q[DCACHE_TAG_HIGH-1:DCACHE_TAG_LOW]),
                                     .Q  (dc_tag_data_raw[i][20:0])
                                    );

         assign w_tout[i][31:DCACHE_TAG_HIGH] = dc_tag_data_raw[i][31-DCACHE_TAG_HIGH:0] ;
         assign w_tout[i][32]                 = dc_tag_data_raw[i][20] ;

         rveven_paritycheck #(32-DCACHE_TAG_HIGH) parcheck(.data_in   (w_tout[i][31:DCACHE_TAG_HIGH]),
                                                   .parity_in (w_tout[i][32]),
                                                   .parity_err(dc_tag_way_perr[i]));
      `endif

   end // block: DCACHE_SZ_16

   else begin : tag_not_64
    `ifdef RV_DCACHE_ECC
     `RV_DCACHE_TAG_CELL  dc_way_tag (
                                     .CLK(dc_tag_clk[i]),
                                     .WE (dc_tag_wren_q[i]),
                                     .D  (dc_tag_wr_data[24:0]),
                                     .ADR(dc_rw_addr_q[DCACHE_TAG_HIGH-1:DCACHE_TAG_LOW]),
                                     .Q  (dc_tag_data_raw[i][24:0])
                                    );

         assign w_tout[i][31:DCACHE_TAG_HIGH] = dc_tag_data_raw[i][31-DCACHE_TAG_HIGH:0] ;
         assign w_tout[i][36:32]              = dc_tag_data_raw[i][24:20] ;

         rvecc_decode  ecc_decode (
                           .en(~dec_tlu_core_ecc_disable),
                           .sed_ded ( 1'b1 ), // 1 : if only need detection
                           .din({12'b0,dc_tag_data_raw[i][19:0]}),
                           .ecc_in({2'b0, dc_tag_data_raw[i][24:20]}),
                           .dout(dc_tag_corrected_data_unc[i][31:0]),
                           .ecc_out(dc_tag_corrected_ecc_unc[i][6:0]),
                           .single_ecc_error(dc_tag_single_ecc_error[i]),
                           .double_ecc_error(dc_tag_double_ecc_error[i]));

          assign dc_tag_way_perr[i]= dc_tag_single_ecc_error[i] | dc_tag_double_ecc_error[i]  ;

     `else
        `RV_DCACHE_TAG_CELL  dc_way_tag (
                                     .CLK(dc_tag_clk[i]),
                                     .WE (dc_tag_wren_q[i]),
                                     .D  (dc_tag_wr_data[20:0]),
                                     .ADR(dc_rw_addr_q[DCACHE_TAG_HIGH-1:DCACHE_TAG_LOW]),
                                     .Q  ({dc_tag_data_raw[i][20:0]})
                                    );

         assign w_tout[i][31:DCACHE_TAG_HIGH] = dc_tag_data_raw[i][31-DCACHE_TAG_HIGH:0] ;
         assign w_tout[i][32]                 = dc_tag_data_raw[i][20] ;

       rveven_paritycheck #(32-DCACHE_TAG_HIGH) parcheck(.data_in   (w_tout[i][31:DCACHE_TAG_HIGH]),
                                                   .parity_in (w_tout[i][32]),
                                                   .parity_err(dc_tag_way_perr[i]));

      `endif
   end // block: tag_not_64
end // block: WAYS


`ifdef RV_DCACHE_ECC
   assign dctag_debug_rd_data[24:0] =  ({25{dc_debug_rd_way_en_ff[0]}} &  dc_tag_data_raw[0] ) |
                                       ({25{dc_debug_rd_way_en_ff[1]}} &  dc_tag_data_raw[1] ) |
                                       ({25{dc_debug_rd_way_en_ff[2]}} &  dc_tag_data_raw[2] ) |
                                       ({25{dc_debug_rd_way_en_ff[3]}} &  dc_tag_data_raw[3] ) ;

`else
   assign dctag_debug_rd_data[20:0] =  ({21{dc_debug_rd_way_en_ff[0]}} &  dc_tag_data_raw[0] ) |
                                       ({21{dc_debug_rd_way_en_ff[1]}} &  dc_tag_data_raw[1] ) |
                                       ({21{dc_debug_rd_way_en_ff[2]}} &  dc_tag_data_raw[2] ) |
                                       ({21{dc_debug_rd_way_en_ff[3]}} &  dc_tag_data_raw[3] ) ;

`endif
   assign dc_rd_hit[0] = (w_tout[0][31:DCACHE_TAG_HIGH] == dc_rw_addr_ff[31:DCACHE_TAG_HIGH]) & dc_tag_valid[0];
   assign dc_rd_hit[1] = (w_tout[1][31:DCACHE_TAG_HIGH] == dc_rw_addr_ff[31:DCACHE_TAG_HIGH]) & dc_tag_valid[1];
   assign dc_rd_hit[2] = (w_tout[2][31:DCACHE_TAG_HIGH] == dc_rw_addr_ff[31:DCACHE_TAG_HIGH]) & dc_tag_valid[2];
   assign dc_rd_hit[3] = (w_tout[3][31:DCACHE_TAG_HIGH] == dc_rw_addr_ff[31:DCACHE_TAG_HIGH]) & dc_tag_valid[3];

   assign  dc_tag_perr  = | (dc_tag_way_perr[3:0] & dc_tag_valid[3:0] ) ;
endmodule