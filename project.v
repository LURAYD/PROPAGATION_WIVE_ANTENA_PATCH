/*
 * Copyright (c) 2024 Uri Shaked
 * SPDX-License-Identifier: Apache-2.0
 * Base: plantilla VGA de Tiny Tapeout (hvsync_generator).
 *
 * Diagrama de radiacion de un arreglo plano de parches microstrip 8 x 10
 * con barrido electronico del haz (phased array). El haz gira en azimut.
 *
 * Modelo fisico (multiplicacion de patrones, Balanis cap. 6 y 14):
 *   u = sin(theta) cos(phi),  v = sin(theta) sin(phi)       (cosenos directores)
 *   F(u,v)   = EF(u,v) * AF(u,v)
 *   AF(u,v)  = D8(psi_x) * D10(psi_y),  DM(psi) = sin(M psi/2) / (M sin(psi/2))
 *   psi_x    = k d u + beta_x,          psi_y = k d v + beta_y
 *   beta_x   = -k d u0,                 beta_y = -k d v0   (fase progresiva)
 *   (u0, v0) = sin(theta0) (cos(phi0), sin(phi0))           (direccion del haz)
 *   |EF|^2   = cos^2(theta) = 1 - u^2 - v^2                  (parche, aprox. cos)
 *   G[dB]    = 20log|D8| + 20log|D10| + 10log(1 - u^2 - v^2)
 *
 * Mapeo a pantalla (vista desde arriba del espacio u-v):
 *   centro (232, 240), radio R = 204.9 px  <->  sin(theta) = 1 (horizonte)
 *   fase de 11 bits: 2048 unidades = 2*pi
 *   k d / R = 1024 / 204.8 = 5 unidades/px (d = lambda/2), 10 unidades/px (d = lambda)
 *   La ganancia se suma en dB (pasos de 0.5 dB) y se pinta de 0 a -31 dB.
 *
 * Superposiciones: circulos theta = 30, 60 y 90 grados, ejes u/v,
 * contorno de -3 dB (blanco) y cruz negra en la direccion comandada (u0, v0).
 * Panel derecho: el arreglo 8 x 10, cada parche coloreado con su fase de
 * excitacion m*beta_x + n*beta_y; debajo, leyenda de fase y escala en dB
 * (marcas en -3, -10, -20 y -30 dB).
 *
 * Entradas:
 *   ui_in[0]   pausa del giro
 *   ui_in[2:1] velocidad (1 a 4 pasos por cuadro, 402 pasos por vuelta)
 *   ui_in[4:3] angulo de barrido theta0: 00 = 30.0, 01 = 48.6, 10 = 14.5, 11 = 0 (broadside)
 *   ui_in[5]   separacion: 0 = lambda/2, 1 = lambda (aparecen lobulos de rejilla)
 *   ui_in[6]   ocultar rejilla, contorno y marcador
 *
 * Simplificaciones: excitacion uniforme, sin acoplamiento mutuo, fases ideales
 * (no cuantizadas), elemento modelado como cos(theta).
 */
`default_nettype none

module tt_um_vga_example (
  input wire [7:0] ui_in,
  output wire [7:0] uo_out,
  input wire [7:0] uio_in,
  output wire [7:0] uio_out,
  output wire [7:0] uio_oe,
  input wire ena,
  input wire clk,
  input wire rst_n
);
  assign uio_out = 8'b0;
  assign uio_oe = 8'b0;

  wire hsync, vsync, video_active;
  wire [9:0] pix_x, pix_y;
  hvsync_generator hvsync_gen (
    .clk(clk), .reset(~rst_n),
    .hsync(hsync), .vsync(vsync), .display_on(video_active),
    .hpos(pix_x), .vpos(pix_y)
  );

  // ---------------------------------------------------------------------
  // Controles sincronizados
  // ---------------------------------------------------------------------
  reg [6:0] ctl_meta, ctl;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ctl_meta <= 7'b0;
      ctl <= 7'b0;
    end else begin
      ctl_meta <= ui_in[6:0];
      ctl <= ctl_meta;
    end
  end
  wire       pause     = ctl[0];
  wire [1:0] speed     = ctl[2:1];
  wire [1:0] theta_sel = ctl[4:3];
  wire       d_lambda  = ctl[5];
  wire       hide_grid = ctl[6];

  // ---------------------------------------------------------------------
  // Azimut del haz: oscilador de Minsky (rotacion sin tablas de seno).
  // osc_c = 1024 cos(phi0), osc_s = 1024 sin(phi0); 1/64 rad por paso.
  // Se actualiza en el borrado vertical (fila 480), 1 a 4 veces por cuadro.
  // ---------------------------------------------------------------------
  reg signed [12:0] osc_c, osc_s;
  wire step = !pause && (pix_y == 10'd480) && (pix_x <= {8'd0, speed});
  wire signed [12:0] osc_c_next = osc_c - ((osc_s + 13'sd32) >>> 6);
  wire signed [12:0] osc_s_next = osc_s + ((osc_c_next + 13'sd32) >>> 6);
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      osc_c <= 13'sd1024;
      osc_s <= 13'sd0;
    end else if (step) begin
      osc_c <= osc_c_next;
      osc_s <= osc_s_next;
    end
  end

  // sin(theta0) * v con sumas de desplazamientos
  function signed [12:0] scale_sin;
    input signed [12:0] v;
    input [1:0] sel;
    begin
      case (sel)
        2'd0: scale_sin = v >>> 1;                 // 0.50 -> 30.0 grados
        2'd1: scale_sin = (v >>> 1) + (v >>> 2);   // 0.75 -> 48.6 grados
        2'd2: scale_sin = v >>> 2;                 // 0.25 -> 14.5 grados
        default: scale_sin = 13'sd0;               // broadside
      endcase
    end
  endfunction

  // 1024*u0, 1024*v0 y desfase progresivo beta = -k d (u0, v0) en unidades de 2*pi/2048
  wire signed [12:0] u0k = scale_sin(osc_c, theta_sel);
  wire signed [12:0] v0k = scale_sin(osc_s, theta_sel);
  wire [10:0] beta_x_h = -u0k[10:0];
  wire [10:0] beta_y_h = -v0k[10:0];
  wire [10:0] beta_x = d_lambda ? {beta_x_h[9:0], 1'b0} : beta_x_h;
  wire [10:0] beta_y = d_lambda ? {beta_y_h[9:0], 1'b0} : beta_y_h;

  // ---------------------------------------------------------------------
  // Tablas calculadas a partir de las formulas (generadas con Python)
  // ---------------------------------------------------------------------
  // -20*log10|sin(8*psi/2)/(8*sin(psi/2))| en pasos de 0.5 dB, psi = pi*(8*i+4)/1024, tope 31.5 dB
  function [5:0] af8_db;
    input [6:0] i;
    begin
      case (i)
        7'd0: af8_db = 6'd0; 7'd1: af8_db = 6'd0; 7'd2: af8_db = 6'd0; 7'd3: af8_db = 6'd0;
        7'd4: af8_db = 6'd1; 7'd5: af8_db = 6'd1; 7'd6: af8_db = 6'd1; 7'd7: af8_db = 6'd2;
        7'd8: af8_db = 6'd2; 7'd9: af8_db = 6'd3; 7'd10: af8_db = 6'd3; 7'd11: af8_db = 6'd4;
        7'd12: af8_db = 6'd5; 7'd13: af8_db = 6'd5; 7'd14: af8_db = 6'd6; 7'd15: af8_db = 6'd7;
        7'd16: af8_db = 6'd8; 7'd17: af8_db = 6'd9; 7'd18: af8_db = 6'd11; 7'd19: af8_db = 6'd12;
        7'd20: af8_db = 6'd14; 7'd21: af8_db = 6'd15; 7'd22: af8_db = 6'd17; 7'd23: af8_db = 6'd19;
        7'd24: af8_db = 6'd22; 7'd25: af8_db = 6'd25; 7'd26: af8_db = 6'd28; 7'd27: af8_db = 6'd32;
        7'd28: af8_db = 6'd36; 7'd29: af8_db = 6'd43; 7'd30: af8_db = 6'd52; 7'd31: af8_db = 6'd63;
        7'd32: af8_db = 6'd63; 7'd33: af8_db = 6'd54; 7'd34: af8_db = 6'd45; 7'd35: af8_db = 6'd40;
        7'd36: af8_db = 6'd36; 7'd37: af8_db = 6'd34; 7'd38: af8_db = 6'd31; 7'd39: af8_db = 6'd30;
        7'd40: af8_db = 6'd28; 7'd41: af8_db = 6'd27; 7'd42: af8_db = 6'd27; 7'd43: af8_db = 6'd26;
        7'd44: af8_db = 6'd26; 7'd45: af8_db = 6'd26; 7'd46: af8_db = 6'd26; 7'd47: af8_db = 6'd26;
        7'd48: af8_db = 6'd26; 7'd49: af8_db = 6'd27; 7'd50: af8_db = 6'd27; 7'd51: af8_db = 6'd28;
        7'd52: af8_db = 6'd29; 7'd53: af8_db = 6'd30; 7'd54: af8_db = 6'd32; 7'd55: af8_db = 6'd33;
        7'd56: af8_db = 6'd35; 7'd57: af8_db = 6'd38; 7'd58: af8_db = 6'd40; 7'd59: af8_db = 6'd44;
        7'd60: af8_db = 6'd48; 7'd61: af8_db = 6'd54; 7'd62: af8_db = 6'd63; 7'd63: af8_db = 6'd63;
        7'd64: af8_db = 6'd63; 7'd65: af8_db = 6'd63; 7'd66: af8_db = 6'd55; 7'd67: af8_db = 6'd50;
        7'd68: af8_db = 6'd46; 7'd69: af8_db = 6'd43; 7'd70: af8_db = 6'd40; 7'd71: af8_db = 6'd38;
        7'd72: af8_db = 6'd37; 7'd73: af8_db = 6'd36; 7'd74: af8_db = 6'd35; 7'd75: af8_db = 6'd34;
        7'd76: af8_db = 6'd33; 7'd77: af8_db = 6'd33; 7'd78: af8_db = 6'd33; 7'd79: af8_db = 6'd33;
        7'd80: af8_db = 6'd33; 7'd81: af8_db = 6'd33; 7'd82: af8_db = 6'd34; 7'd83: af8_db = 6'd34;
        7'd84: af8_db = 6'd35; 7'd85: af8_db = 6'd36; 7'd86: af8_db = 6'd38; 7'd87: af8_db = 6'd39;
        7'd88: af8_db = 6'd41; 7'd89: af8_db = 6'd43; 7'd90: af8_db = 6'd46; 7'd91: af8_db = 6'd49;
        7'd92: af8_db = 6'd53; 7'd93: af8_db = 6'd59; 7'd94: af8_db = 6'd63; 7'd95: af8_db = 6'd63;
        7'd96: af8_db = 6'd63; 7'd97: af8_db = 6'd63; 7'd98: af8_db = 6'd60; 7'd99: af8_db = 6'd54;
        7'd100: af8_db = 6'd50; 7'd101: af8_db = 6'd47; 7'd102: af8_db = 6'd44; 7'd103: af8_db = 6'd42;
        7'd104: af8_db = 6'd41; 7'd105: af8_db = 6'd39; 7'd106: af8_db = 6'd38; 7'd107: af8_db = 6'd37;
        7'd108: af8_db = 6'd37; 7'd109: af8_db = 6'd36; 7'd110: af8_db = 6'd36; 7'd111: af8_db = 6'd36;
        7'd112: af8_db = 6'd36; 7'd113: af8_db = 6'd36; 7'd114: af8_db = 6'd36; 7'd115: af8_db = 6'd37;
        7'd116: af8_db = 6'd38; 7'd117: af8_db = 6'd39; 7'd118: af8_db = 6'd40; 7'd119: af8_db = 6'd41;
        7'd120: af8_db = 6'd43; 7'd121: af8_db = 6'd45; 7'd122: af8_db = 6'd48; 7'd123: af8_db = 6'd51;
        7'd124: af8_db = 6'd55; 7'd125: af8_db = 6'd61; 7'd126: af8_db = 6'd63; 7'd127: af8_db = 6'd63;
        default: af8_db = 6'd63;
      endcase
    end
  endfunction

  // -20*log10|sin(10*psi/2)/(10*sin(psi/2))| en pasos de 0.5 dB, misma malla de psi
  function [5:0] af10_db;
    input [6:0] i;
    begin
      case (i)
        7'd0: af10_db = 6'd0; 7'd1: af10_db = 6'd0; 7'd2: af10_db = 6'd0; 7'd3: af10_db = 6'd1;
        7'd4: af10_db = 6'd1; 7'd5: af10_db = 6'd1; 7'd6: af10_db = 6'd2; 7'd7: af10_db = 6'd3;
        7'd8: af10_db = 6'd3; 7'd9: af10_db = 6'd4; 7'd10: af10_db = 6'd5; 7'd11: af10_db = 6'd6;
        7'd12: af10_db = 6'd7; 7'd13: af10_db = 6'd9; 7'd14: af10_db = 6'd10; 7'd15: af10_db = 6'd12;
        7'd16: af10_db = 6'd14; 7'd17: af10_db = 6'd16; 7'd18: af10_db = 6'd19; 7'd19: af10_db = 6'd22;
        7'd20: af10_db = 6'd25; 7'd21: af10_db = 6'd29; 7'd22: af10_db = 6'd35; 7'd23: af10_db = 6'd42;
        7'd24: af10_db = 6'd54; 7'd25: af10_db = 6'd63; 7'd26: af10_db = 6'd58; 7'd27: af10_db = 6'd46;
        7'd28: af10_db = 6'd40; 7'd29: af10_db = 6'd35; 7'd30: af10_db = 6'd32; 7'd31: af10_db = 6'd30;
        7'd32: af10_db = 6'd29; 7'd33: af10_db = 6'd27; 7'd34: af10_db = 6'd27; 7'd35: af10_db = 6'd26;
        7'd36: af10_db = 6'd26; 7'd37: af10_db = 6'd26; 7'd38: af10_db = 6'd26; 7'd39: af10_db = 6'd27;
        7'd40: af10_db = 6'd28; 7'd41: af10_db = 6'd29; 7'd42: af10_db = 6'd30; 7'd43: af10_db = 6'd32;
        7'd44: af10_db = 6'd34; 7'd45: af10_db = 6'd37; 7'd46: af10_db = 6'd40; 7'd47: af10_db = 6'd44;
        7'd48: af10_db = 6'd49; 7'd49: af10_db = 6'd58; 7'd50: af10_db = 6'd63; 7'd51: af10_db = 6'd63;
        7'd52: af10_db = 6'd63; 7'd53: af10_db = 6'd54; 7'd54: af10_db = 6'd48; 7'd55: af10_db = 6'd44;
        7'd56: af10_db = 6'd41; 7'd57: af10_db = 6'd39; 7'd58: af10_db = 6'd37; 7'd59: af10_db = 6'd36;
        7'd60: af10_db = 6'd35; 7'd61: af10_db = 6'd34; 7'd62: af10_db = 6'd34; 7'd63: af10_db = 6'd34;
        7'd64: af10_db = 6'd34; 7'd65: af10_db = 6'd35; 7'd66: af10_db = 6'd35; 7'd67: af10_db = 6'd36;
        7'd68: af10_db = 6'd38; 7'd69: af10_db = 6'd39; 7'd70: af10_db = 6'd41; 7'd71: af10_db = 6'd44;
        7'd72: af10_db = 6'd48; 7'd73: af10_db = 6'd52; 7'd74: af10_db = 6'd58; 7'd75: af10_db = 6'd63;
        7'd76: af10_db = 6'd63; 7'd77: af10_db = 6'd63; 7'd78: af10_db = 6'd63; 7'd79: af10_db = 6'd56;
        7'd80: af10_db = 6'd51; 7'd81: af10_db = 6'd48; 7'd82: af10_db = 6'd45; 7'd83: af10_db = 6'd43;
        7'd84: af10_db = 6'd41; 7'd85: af10_db = 6'd40; 7'd86: af10_db = 6'd39; 7'd87: af10_db = 6'd38;
        7'd88: af10_db = 6'd38; 7'd89: af10_db = 6'd38; 7'd90: af10_db = 6'd38; 7'd91: af10_db = 6'd39;
        7'd92: af10_db = 6'd39; 7'd93: af10_db = 6'd40; 7'd94: af10_db = 6'd42; 7'd95: af10_db = 6'd44;
        7'd96: af10_db = 6'd46; 7'd97: af10_db = 6'd49; 7'd98: af10_db = 6'd52; 7'd99: af10_db = 6'd57;
        7'd100: af10_db = 6'd63; 7'd101: af10_db = 6'd63; 7'd102: af10_db = 6'd63; 7'd103: af10_db = 6'd63;
        7'd104: af10_db = 6'd63; 7'd105: af10_db = 6'd57; 7'd106: af10_db = 6'd52; 7'd107: af10_db = 6'd49;
        7'd108: af10_db = 6'd46; 7'd109: af10_db = 6'd44; 7'd110: af10_db = 6'd43; 7'd111: af10_db = 6'd41;
        7'd112: af10_db = 6'd41; 7'd113: af10_db = 6'd40; 7'd114: af10_db = 6'd40; 7'd115: af10_db = 6'd40;
        7'd116: af10_db = 6'd40; 7'd117: af10_db = 6'd41; 7'd118: af10_db = 6'd41; 7'd119: af10_db = 6'd42;
        7'd120: af10_db = 6'd44; 7'd121: af10_db = 6'd46; 7'd122: af10_db = 6'd48; 7'd123: af10_db = 6'd51;
        7'd124: af10_db = 6'd55; 7'd125: af10_db = 6'd61; 7'd126: af10_db = 6'd63; 7'd127: af10_db = 6'd63;
        default: af10_db = 6'd63;
      endcase
    end
  endfunction

  // -10*log10(1 - s), s = sin^2(theta) = (i+0.5)*1024/41984: patron cos(theta) del parche
  function [5:0] elem_db;
    input [5:0] i;
    begin
      case (i)
        6'd0: elem_db = 6'd0; 6'd1: elem_db = 6'd0; 6'd2: elem_db = 6'd1; 6'd3: elem_db = 6'd1;
        6'd4: elem_db = 6'd1; 6'd5: elem_db = 6'd1; 6'd6: elem_db = 6'd1; 6'd7: elem_db = 6'd2;
        6'd8: elem_db = 6'd2; 6'd9: elem_db = 6'd2; 6'd10: elem_db = 6'd3; 6'd11: elem_db = 6'd3;
        6'd12: elem_db = 6'd3; 6'd13: elem_db = 6'd3; 6'd14: elem_db = 6'd4; 6'd15: elem_db = 6'd4;
        6'd16: elem_db = 6'd4; 6'd17: elem_db = 6'd5; 6'd18: elem_db = 6'd5; 6'd19: elem_db = 6'd6;
        6'd20: elem_db = 6'd6; 6'd21: elem_db = 6'd6; 6'd22: elem_db = 6'd7; 6'd23: elem_db = 6'd7;
        6'd24: elem_db = 6'd8; 6'd25: elem_db = 6'd8; 6'd26: elem_db = 6'd9; 6'd27: elem_db = 6'd10;
        6'd28: elem_db = 6'd10; 6'd29: elem_db = 6'd11; 6'd30: elem_db = 6'd12; 6'd31: elem_db = 6'd13;
        6'd32: elem_db = 6'd14; 6'd33: elem_db = 6'd15; 6'd34: elem_db = 6'd16; 6'd35: elem_db = 6'd17;
        6'd36: elem_db = 6'd19; 6'd37: elem_db = 6'd21; 6'd38: elem_db = 6'd24; 6'd39: elem_db = 6'd29;
        6'd40: elem_db = 6'd38;
        default: elem_db = 6'd63;
      endcase
    end
  endfunction

  // ---------------------------------------------------------------------
  // Etapa 0: fases psi por pixel, tablas del factor de arreglo y r^2
  // ---------------------------------------------------------------------
  wire signed [10:0] dx = $signed({1'b0, pix_x}) - 11'sd232;
  wire signed [10:0] dy = 11'sd240 - $signed({1'b0, pix_y});   // v positivo hacia arriba
  wire [10:0] dx5 = {dx[8:0], 2'b00} + dx;                      // 5*dx mod 2048
  wire [10:0] dy5 = {dy[8:0], 2'b00} + dy;
  wire [10:0] kdu = d_lambda ? {dx5[9:0], 1'b0} : dx5;          // k d u
  wire [10:0] kdv = d_lambda ? {dy5[9:0], 1'b0} : dy5;
  wire [10:0] psi_x = kdu + beta_x;
  wire [10:0] psi_y = kdv + beta_y;
  // |DM(psi)| = |DM(2 pi - psi)|: se pliega a [0, pi) y se indexa con 7 bits
  wire [9:0] fold_x = psi_x[10] ? ~psi_x[9:0] : psi_x[9:0];
  wire [9:0] fold_y = psi_y[10] ? ~psi_y[9:0] : psi_y[9:0];
  wire [5:0] gx_db = af8_db(fold_x[9:3]);
  wire [5:0] gy_db = af10_db(fold_y[9:3]);

  wire [10:0] dx_abs = dx[10] ? -dx : dx;
  wire [10:0] dy_abs = dy[10] ? -dy : dy;
  wire [17:0] dx2 = dx_abs[8:0] * dx_abs[8:0];
  wire [17:0] dy2 = dy_abs[8:0] * dy_abs[8:0];
  wire [18:0] r2 = dx2 + dy2;                                   // (R sin(theta))^2

  // Registro de etapa 1
  reg [5:0] s1_gx, s1_gy;
  reg [18:0] s1_r2;
  reg [9:0] s1_x, s1_y;
  reg s1_hs, s1_vs, s1_de;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s1_gx <= 6'd0; s1_gy <= 6'd0; s1_r2 <= 19'd0;
      s1_x <= 10'd0; s1_y <= 10'd0;
      s1_hs <= 1'b0; s1_vs <= 1'b0; s1_de <= 1'b0;
    end else begin
      s1_gx <= gx_db; s1_gy <= gy_db; s1_r2 <= r2;
      s1_x <= pix_x; s1_y <= pix_y;
      s1_hs <= hsync; s1_vs <= vsync; s1_de <= video_active;
    end
  end

  // ---------------------------------------------------------------------
  // Etapa 1: ganancia total, superposiciones, panel del arreglo y leyendas
  // ---------------------------------------------------------------------
  wire visible = (s1_r2 < 19'd41984);                           // sin(theta) < 1
  wire [5:0] ge_db = elem_db(s1_r2[15:10]);
  wire [7:0] g_sum = s1_gx + s1_gy + ge_db;                     // atenuacion total, 0.5 dB
  wire [5:0] g_clip = (g_sum > 8'd63) ? 6'd63 : g_sum[5:0];
  wire [4:0] level = 5'd31 - g_clip[5:1];                       // 31 = 0 dB, 0 = -31 dB
  wire contour = (g_clip == 6'd6);                              // -3 dB
  wire ring = ((s1_r2 >= 19'd10394) && (s1_r2 <= 19'd10598)) ||  // theta = 30
              ((s1_r2 >= 19'd31311) && (s1_r2 <= 19'd31665));    // theta = 60
  wire rim = (s1_r2 >= 19'd41164);                              // theta = 90
  wire axis = (s1_x == 10'd232) || (s1_y == 10'd240);

  // Direccion comandada en pixeles: R*u0 = 1024*u0 * 204.9/1024 ~ (u0k * 51) >> 8
  wire signed [19:0] mk_u = u0k * 20'sd51;
  wire signed [19:0] mk_v = v0k * 20'sd51;
  wire signed [11:0] mk_x = 12'sd232 + mk_u[19:8];
  wire signed [11:0] mk_y = 12'sd240 - mk_v[19:8];
  wire signed [11:0] mdx = $signed({2'b0, s1_x}) - mk_x;
  wire signed [11:0] mdy = $signed({2'b0, s1_y}) - mk_y;
  wire [11:0] amdx = mdx[11] ? -mdx : mdx;
  wire [11:0] amdy = mdy[11] ? -mdy : mdy;
  wire marker = ((mdx == 12'sd0) && (amdy >= 12'd2) && (amdy <= 12'd6)) ||
                ((mdy == 12'sd0) && (amdx >= 12'd2) && (amdx <= 12'd6));

  // Panel del arreglo: 8 columnas (m, eje u) x 10 filas (n, eje v), celdas de 16 px
  wire [9:0] ax = s1_x - 10'd472;
  wire [9:0] ay = s1_y - 10'd48;
  wire in_sub = (s1_x >= 10'd464) && (s1_x < 10'd608) && (s1_y >= 10'd40) && (s1_y < 10'd216);
  wire in_arr = (ax < 10'd128) && (ay < 10'd160);
  wire [3:0] lx = ax[3:0];
  wire [3:0] ly = ay[3:0];
  wire patch = (lx >= 4'd3) && (lx <= 4'd12) && (ly >= 4'd2) && (ly <= 4'd11);
  wire feed = ((lx == 4'd7) || (lx == 4'd8)) && (ly >= 4'd12);
  wire [2:0] el_m = ax[6:4];
  wire [3:0] el_n = 4'd9 - ay[7:4];
  wire [10:0] el_phase = el_m * beta_x + el_n * beta_y;         // fase de excitacion, mod 2*pi

  wire hue_bar = (ax < 10'd128) && (s1_y >= 10'd222) && (s1_y < 10'd230);
  wire [9:0] by = s1_y - 10'd248;
  wire db_bar = (s1_x >= 10'd476) && (s1_x < 10'd492) && (by < 10'd128);
  wire db_tick = (s1_x >= 10'd494) && (s1_x < 10'd502) &&
                 ((s1_y == 10'd260) || (s1_y == 10'd288) || (s1_y == 10'd328) || (s1_y == 10'd368));

  // Paleta de falso color: 0 = azul (-31 dB) ... 31 = rojo (0 dB)
  function [11:0] heat_color;
    input [4:0] lv;
    reg [3:0] a;
    begin
      a = {lv[2:0], 1'b0};
      case (lv[4:3])
        2'd0: heat_color = {4'd3, a, 4'd15};
        2'd1: heat_color = {4'd0, 4'd15, (4'd14 - a)};
        2'd2: heat_color = {a, 4'd15, 4'd0};
        default: heat_color = {4'd15, (4'd14 - a), 4'd0};
      endcase
    end
  endfunction

  // Rueda de color para la fase (16 pasos de 22.5 grados)
  function [11:0] hue16;
    input [3:0] h;
    begin
      case (h)
        4'd0: hue16 = 12'hf00;  4'd1: hue16 = 12'hf50;  4'd2: hue16 = 12'hfa0;  4'd3: hue16 = 12'hff0;
        4'd4: hue16 = 12'haf0;  4'd5: hue16 = 12'h5f0;  4'd6: hue16 = 12'h0f0;  4'd7: hue16 = 12'h0f8;
        4'd8: hue16 = 12'h0ff;  4'd9: hue16 = 12'h08f;  4'd10: hue16 = 12'h00f; 4'd11: hue16 = 12'h50f;
        4'd12: hue16 = 12'ha0f; 4'd13: hue16 = 12'hf0f; 4'd14: hue16 = 12'hf08; default: hue16 = 12'hf04;
      endcase
    end
  endfunction

  reg [11:0] rgb;
  reg [11:0] heat;
  always @* begin
    heat = heat_color(level);
    rgb = 12'h112;
    if (visible) begin
      rgb = heat;
      if (!hide_grid) begin
        if (ring || axis) rgb = {1'b1, heat[11:9], 1'b1, heat[7:5], 1'b1, heat[3:1]};
        if (contour) rgb = 12'hfff;
        if (marker) rgb = 12'h000;
      end
      if (rim) rgb = 12'hccc;
    end
    if (in_sub) rgb = 12'h134;
    if (in_arr) begin
      if (patch) rgb = hue16(el_phase[10:7]);
      else if (feed) rgb = 12'hc84;
    end
    if (hue_bar) rgb = hue16(ax[6:3]);
    if (db_bar) rgb = heat_color(5'd31 - by[6:2]);
    if (db_tick) rgb = 12'hfff;
  end

  // Registro de etapa 2
  reg [11:0] s2_rgb;
  reg [1:0] s2_lsb;
  reg s2_hs, s2_vs, s2_de;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s2_rgb <= 12'd0; s2_lsb <= 2'd0;
      s2_hs <= 1'b0; s2_vs <= 1'b0; s2_de <= 1'b0;
    end else begin
      s2_rgb <= rgb; s2_lsb <= {s1_x[0], s1_y[0]};
      s2_hs <= s1_hs; s2_vs <= s1_vs; s2_de <= s1_de;
    end
  end

  // ---------------------------------------------------------------------
  // Etapa 2: tramado ordenado 2 x 2 a 2 bits por canal (TinyVGA) y salida
  // ---------------------------------------------------------------------
  wire [1:0] threshold = {s2_lsb[1], 1'b0} ^ (s2_lsb[0] ? 2'd3 : 2'd0);
  function [1:0] quantize;
    input [3:0] value;
    input [1:0] threshold_value;
    begin
      if ((value[3:2] != 2'd3) && (value[1:0] > threshold_value))
        quantize = value[3:2] + 2'd1;
      else quantize = value[3:2];
    end
  endfunction
  wire [1:0] R = s2_de ? quantize(s2_rgb[11:8], threshold) : 2'd0;
  wire [1:0] G = s2_de ? quantize(s2_rgb[7:4], threshold) : 2'd0;
  wire [1:0] B = s2_de ? quantize(s2_rgb[3:0], threshold) : 2'd0;

  reg [7:0] uo_q;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) uo_q <= 8'b0;
    else uo_q <= {s2_hs, B[0], G[0], R[0], s2_vs, B[1], G[1], R[1]};
  end
  assign uo_out = uo_q;

  wire _unused_ok = &{1'b0, ena, uio_in, ui_in[7], el_phase[6:0], dx_abs[10:9], dy_abs[10:9],
                      mk_u[7:0], mk_v[7:0], fold_x[2:0], fold_y[2:0]};
endmodule

`default_nettype wire