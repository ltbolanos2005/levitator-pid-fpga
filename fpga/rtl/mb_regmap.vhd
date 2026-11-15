-- =====================================================================
-- mb_regmap.vhd
-- Traductor entre el mapa MODBUS (el contrato con el ESP32 / HMI) y el
-- mapa interno del levitador (reg_bank: mm y ganancias x/4096).
-- Ver la tabla de unidades en lev_pkg.
--
--  Escritura (MODBUS -> reg_bank, puerto 2):
--     SP  -> SP_mm  = round(SP / 10)
--     KP  -> KP_int = round(KP * 4.096)      KI -> round(KI * 0.2048)
--     KD  -> KD_int = round(KD * 81.92)      EN -> ENABLE   BIAS -> BIAS
--  Lectura (reg_bank -> MODBUS):
--     PV  = PV_mm * 10,  ERR = -(PV-SP)_mm * 10 = SP - PV,  PWM = MV,
--     STATUS, EN y BIAS tal cual.
--     SP/KP/KI/KD: se devuelve EXACTAMENTE lo ultimo que escribio el
--     maestro mientras el registro interno no haya cambiado por otra via
--     (p. ej. la consola del Arduino).  Si cambio, se devuelve la
--     conversion inversa del valor interno.  Asi la HMI ve "0.79" cuando
--     escribio 0.79, aunque internamente KI se redondee a 16/4096.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.lev_pkg.all;

entity mb_regmap is
    port (
        clk      : in  std_logic;
        rst_n    : in  std_logic;
        -- Lado MODBUS
        mb_wr_en   : in  std_logic;
        mb_wr_addr : in  integer range 0 to MB_NUM_REGS - 1;
        mb_wr_data : in  word_t;
        mb_regs    : out mb_array_t;
        -- Lado interno
        regs       : in  reg_array_t;
        wr_en      : out std_logic;
        wr_addr    : out integer range 0 to NUM_REGS - 1;
        wr_data    : out word_t
    );
end entity mb_regmap;

architecture rtl of mb_regmap is
    -- Copias "sombra" de SP, KP, KI, KD: valor MODBUS escrito y su
    -- equivalente interno.  Indice 0..3 = SP, KP, KI, KD.
    type sh_arr_t is array (0 to 3) of word_t;
    signal sh_mb   : sh_arr_t := (others => (others => '0'));
    signal sh_int  : sh_arr_t := (others => (others => '0'));
    signal sh_ok   : std_logic_vector(3 downto 0) := (others => '0');
    signal out_r   : mb_array_t := (others => (others => '0'));
    constant REFRESH_CLKS : integer := 64;
    signal rr      : integer range 0 to MB_NUM_REGS - 1 := 0;
    signal pre     : integer range 0 to REFRESH_CLKS - 1 := 0;

    type idx_arr_t is array (0 to 3) of integer;
    constant INT_OF : idx_arr_t := (REG_SP, REG_KP, REG_KI, REG_KD);
begin
    mb_regs <= out_r;

    -- -----------------------------------------------------------------
    -- Escritura: conversion y paso al puerto 2 de reg_bank (1 ciclo)
    -- -----------------------------------------------------------------
    process (clk, rst_n)
        variable v : word_t;
        variable s : integer range 0 to 3;
    begin
        if rst_n = '0' then
            wr_en   <= '0';
            wr_addr <= 0;
            wr_data <= (others => '0');
            sh_ok   <= (others => '0');
        elsif rising_edge(clk) then
            wr_en <= '0';
            if mb_wr_en = '1' then
                s := 0;
                case mb_wr_addr is
                    when MB_SP   => v := scale_u16(mb_wr_data, MBK_SP_NUM, MBK_SP_SH); wr_addr <= REG_SP;     s := 0;
                    when MB_KP   => v := scale_u16(mb_wr_data, MBK_KP_NUM, MBK_KP_SH); wr_addr <= REG_KP;     s := 1;
                    when MB_KI   => v := scale_u16(mb_wr_data, MBK_KI_NUM, MBK_KI_SH); wr_addr <= REG_KI;     s := 2;
                    when MB_KD   => v := scale_u16(mb_wr_data, MBK_KD_NUM, MBK_KD_SH); wr_addr <= REG_KD;     s := 3;
                    when MB_EN   => v := mb_wr_data;                                   wr_addr <= REG_ENABLE;
                    when MB_BIAS => v := mb_wr_data;                                   wr_addr <= REG_BIAS;
                    when others  => v := (others => '0');                              wr_addr <= REG_PV;   -- no escribible
                end case;
                wr_data <= v;
                if mb_writable(mb_wr_addr) then
                    wr_en <= '1';
                end if;
                if mb_wr_addr = MB_SP or mb_wr_addr = MB_KP or
                   mb_wr_addr = MB_KI or mb_wr_addr = MB_KD then
                    sh_mb(s)  <= mb_wr_data;
                    sh_int(s) <= v;
                    sh_ok(s)  <= '1';
                end if;
            end if;
        end if;
    end process;

    -- -----------------------------------------------------------------
    -- Lectura: mapa MODBUS registrado.  Se refresca UN registro cada
    -- REFRESH_CLKS ciclos (todo el mapa en 10 x 64 = 640 ciclos = 12.8 us
    -- a 50 MHz, mucho menos que un caracter MODBUS).  Asi se comparte una
    -- sola ruta de conversion en vez de 7 multiplicadores en paralelo.
    -- -----------------------------------------------------------------
    process (clk, rst_n)
        variable inv : word_t;
        variable si  : integer range 0 to 3;
    begin
        if rst_n = '0' then
            out_r <= (others => (others => '0'));
            rr    <= 0;
            pre   <= 0;
        elsif rising_edge(clk) then
            if pre /= REFRESH_CLKS - 1 then
                pre <= pre + 1;
            else
                pre <= 0;
                case rr is
                    when MB_SP | MB_KP | MB_KI | MB_KD =>
                        si := rr;                            -- MB_SP..MB_KD son 0..3
                        case si is
                            when 0      => inv := x10_sat_u(regs(REG_SP));
                            when 1      => inv := scale_u16(regs(REG_KP), MBR_KP_NUM, MBR_KP_SH);
                            when 2      => inv := scale_u16(regs(REG_KI), MBR_KI_NUM, MBR_KI_SH);
                            when others => inv := scale_u16(regs(REG_KD), MBR_KD_NUM, MBR_KD_SH);
                        end case;
                        if sh_ok(si) = '1' and regs(INT_OF(si)) = sh_int(si) then
                            out_r(rr) <= sh_mb(si);          -- lo ultimo que escribio el maestro
                        else
                            out_r(rr) <= inv;                -- cambio por otra via
                        end if;
                    when MB_EN     => out_r(rr) <= regs(REG_ENABLE);
                    when MB_PV     => out_r(rr) <= x10_sat_u(regs(REG_PV));
                    when MB_ERR    => out_r(rr) <= neg_x10_sat_s(regs(REG_ERROR));
                    when MB_PWM    => out_r(rr) <= regs(REG_MV);
                    when MB_STATUS => out_r(rr) <= regs(REG_STATUS);
                    when others    => out_r(rr) <= regs(REG_BIAS);
                end case;
                if rr = MB_NUM_REGS - 1 then rr <= 0; else rr <= rr + 1; end if;
            end if;
        end if;
    end process;

end architecture rtl;
