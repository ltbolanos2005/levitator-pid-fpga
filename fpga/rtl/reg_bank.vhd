-- =====================================================================
-- reg_bank.vhd
-- Banco de registros del levitador (mapa INTERNO de lev_pkg; el ESP32
-- lo ve a traves de mb_regmap).
--  * Registros de solo lectura (PV, MV, ERROR, STATUS): se refrescan
--    cada ciclo desde el lazo de control.
--  * Registros de lectura/escritura (SP, KP, KI, KD, ENABLE, BIAS):
--    se escriben desde dos puertos independientes:
--       puerto 1 = enlace con el Arduino (ya en uso)
--       puerto 2 = esclavo MODBUS RTU (via mb_regmap, ya en unidades internas)
--    Ambos pueden escribir en el mismo ciclo; si es el mismo registro
--    gana el puerto 1.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.lev_pkg.all;

entity reg_bank is
    generic (
        -- Valores al encender / reset (por defecto los de lev_pkg)
        INIT_SP   : integer := DEF_SP;
        INIT_KP   : integer := DEF_KP;
        INIT_KI   : integer := DEF_KI;
        INIT_KD   : integer := DEF_KD;
        INIT_EN   : integer := DEF_ENABLE;
        INIT_BIAS : integer := DEF_BIAS
    );
    port (
        clk       : in  std_logic;
        rst_n     : in  std_logic;
        -- Puerto de escritura 1 (Arduino)
        wr1_en    : in  std_logic;
        wr1_addr  : in  integer range 0 to NUM_REGS - 1;
        wr1_data  : in  word_t;
        -- Puerto de escritura 2 (MODBUS)
        wr2_en    : in  std_logic;
        wr2_addr  : in  integer range 0 to NUM_REGS - 1;
        wr2_data  : in  word_t;
        -- Valores de solo lectura
        pv_in     : in  word_t;
        mv_in     : in  word_t;
        err_in    : in  word_t;
        status_in : in  word_t;
        -- Contenido completo
        regs      : out reg_array_t
    );
end entity reg_bank;

architecture rtl of reg_bank is
    signal r : reg_array_t := (others => (others => '0'));
begin
    regs <= r;

    process (clk, rst_n)
        procedure write_one(wa : integer; wd : word_t) is
        begin
            if reg_writable(wa) then
                if wa = REG_ENABLE then
                    -- Cualquier valor distinto de 0 = lazo habilitado
                    if wd /= x"0000" then r(wa) <= x"0001"; else r(wa) <= x"0000"; end if;
                elsif wa = REG_BIAS then
                    -- El PWM base no puede pasar de 255
                    if unsigned(wd) > 255 then r(wa) <= x"00FF"; else r(wa) <= wd; end if;
                else
                    r(wa) <= wd;
                end if;
            end if;
        end procedure;
    begin
        if rst_n = '0' then
            r             <= (others => (others => '0'));
            r(REG_SP)     <= std_logic_vector(to_unsigned(INIT_SP,   16));
            r(REG_KP)     <= std_logic_vector(to_unsigned(INIT_KP,   16));
            r(REG_KI)     <= std_logic_vector(to_unsigned(INIT_KI,   16));
            r(REG_KD)     <= std_logic_vector(to_unsigned(INIT_KD,   16));
            r(REG_ENABLE) <= std_logic_vector(to_unsigned(INIT_EN, 16));
            r(REG_BIAS)   <= std_logic_vector(to_unsigned(INIT_BIAS, 16));
        elsif rising_edge(clk) then
            -- Solo lectura
            r(REG_PV)     <= pv_in;
            r(REG_MV)     <= mv_in;
            r(REG_ERROR)  <= err_in;
            r(REG_STATUS) <= status_in;

            -- Escrituras: ambos puertos pueden escribir en el MISMO ciclo
            -- (antes el puerto 2 se perdia si coincidia con el 1).  Se aplica
            -- primero el 2 y despues el 1, asi que si apuntan al mismo
            -- registro gana el Arduino (ultima asignacion).
            if wr2_en = '1' then
                write_one(wr2_addr, wr2_data);
            end if;
            if wr1_en = '1' then
                write_one(wr1_addr, wr1_data);
            end if;
        end if;
    end process;

end architecture rtl;
