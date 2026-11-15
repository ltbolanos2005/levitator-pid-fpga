-- =====================================================================
-- uart_rx.vhd
-- Receptor UART 8N1 con sincronizador de 2 etapas y validacion de
-- bit de arranque (centro del bit) y bit de parada.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_rx is
    generic (
        CLKS_PER_BIT : integer := 5208   -- 50 MHz / 9600
    );
    port (
        clk       : in  std_logic;
        rst_n     : in  std_logic;
        rxd       : in  std_logic;
        data      : out std_logic_vector(7 downto 0);
        valid     : out std_logic;   -- pulso de 1 ciclo: byte correcto
        frame_err : out std_logic    -- pulso de 1 ciclo: bit de parada en '0'
    );
end entity uart_rx;

architecture rtl of uart_rx is
    type state_t is (S_IDLE, S_START, S_DATA, S_STOP);
    signal state   : state_t := S_IDLE;
    signal meta    : std_logic := '1';
    signal sync    : std_logic := '1';
    signal clk_cnt : integer range 0 to CLKS_PER_BIT - 1 := 0;
    signal bit_idx : integer range 0 to 7 := 0;
    signal shreg   : std_logic_vector(7 downto 0) := (others => '0');
begin

    process (clk, rst_n)
    begin
        if rst_n = '0' then
            state     <= S_IDLE;
            meta      <= '1';
            sync      <= '1';
            clk_cnt   <= 0;
            bit_idx   <= 0;
            valid     <= '0';
            frame_err <= '0';
            data      <= (others => '0');
        elsif rising_edge(clk) then
            meta      <= rxd;
            sync      <= meta;
            valid     <= '0';
            frame_err <= '0';

            case state is
                when S_IDLE =>
                    if sync = '0' then
                        clk_cnt <= CLKS_PER_BIT / 2;
                        state   <= S_START;
                    end if;

                when S_START =>
                    if clk_cnt = 0 then
                        if sync = '0' then
                            clk_cnt <= CLKS_PER_BIT - 1;
                            bit_idx <= 0;
                            state   <= S_DATA;
                        else
                            state <= S_IDLE;          -- pulso de ruido
                        end if;
                    else
                        clk_cnt <= clk_cnt - 1;
                    end if;

                when S_DATA =>
                    if clk_cnt = 0 then
                        shreg(bit_idx) <= sync;
                        clk_cnt <= CLKS_PER_BIT - 1;
                        if bit_idx = 7 then
                            state <= S_STOP;
                        else
                            bit_idx <= bit_idx + 1;
                        end if;
                    else
                        clk_cnt <= clk_cnt - 1;
                    end if;

                when S_STOP =>
                    if clk_cnt = 0 then
                        if sync = '1' then
                            data  <= shreg;
                            valid <= '1';
                        else
                            frame_err <= '1';
                        end if;
                        state <= S_IDLE;
                    else
                        clk_cnt <= clk_cnt - 1;
                    end if;
            end case;
        end if;
    end process;

end architecture rtl;
