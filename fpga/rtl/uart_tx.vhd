-- =====================================================================
-- uart_tx.vhd
-- Transmisor UART 8N1. busy = '1' desde el ciclo siguiente a start
-- hasta terminar el bit de parada.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_tx is
    generic (
        CLKS_PER_BIT : integer := 5208
    );
    port (
        clk   : in  std_logic;
        rst_n : in  std_logic;
        start : in  std_logic;
        data  : in  std_logic_vector(7 downto 0);
        txd   : out std_logic;
        busy  : out std_logic
    );
end entity uart_tx;

architecture rtl of uart_tx is
    type state_t is (S_IDLE, S_START, S_DATA, S_STOP);
    signal state   : state_t := S_IDLE;
    signal clk_cnt : integer range 0 to CLKS_PER_BIT - 1 := 0;
    signal bit_idx : integer range 0 to 7 := 0;
    signal shreg   : std_logic_vector(7 downto 0) := (others => '0');
    signal txd_i   : std_logic := '1';
    signal busy_i  : std_logic := '0';
begin
    txd  <= txd_i;
    busy <= busy_i;

    process (clk, rst_n)
    begin
        if rst_n = '0' then
            state   <= S_IDLE;
            txd_i   <= '1';
            busy_i  <= '0';
            clk_cnt <= 0;
            bit_idx <= 0;
        elsif rising_edge(clk) then
            case state is
                when S_IDLE =>
                    txd_i  <= '1';
                    busy_i <= '0';
                    if start = '1' then
                        shreg   <= data;
                        busy_i  <= '1';
                        txd_i   <= '0';               -- bit de arranque
                        clk_cnt <= CLKS_PER_BIT - 1;
                        state   <= S_START;
                    end if;

                when S_START =>
                    if clk_cnt = 0 then
                        txd_i   <= shreg(0);
                        bit_idx <= 0;
                        clk_cnt <= CLKS_PER_BIT - 1;
                        state   <= S_DATA;
                    else
                        clk_cnt <= clk_cnt - 1;
                    end if;

                when S_DATA =>
                    if clk_cnt = 0 then
                        clk_cnt <= CLKS_PER_BIT - 1;
                        if bit_idx = 7 then
                            txd_i <= '1';             -- bit de parada
                            state <= S_STOP;
                        else
                            bit_idx <= bit_idx + 1;
                            txd_i   <= shreg(bit_idx + 1);
                        end if;
                    else
                        clk_cnt <= clk_cnt - 1;
                    end if;

                when S_STOP =>
                    if clk_cnt = 0 then
                        busy_i <= '0';
                        state  <= S_IDLE;
                    else
                        clk_cnt <= clk_cnt - 1;
                    end if;
            end case;
        end if;
    end process;

end architecture rtl;
