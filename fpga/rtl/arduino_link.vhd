-- =====================================================================
-- arduino_link.vhd
-- Enlace serie local con el Arduino (9600 8N1). Tramas con byte de
-- sincronismo y checksum XOR (el XOR de todos los bytes, incluido el
-- checksum, debe dar 0).
--
--  Arduino -> FPGA
--    Medicion : AA  PV_H PV_L          CHK   (PV en mm, FFFF = sin eco)
--    Escribir : A5  REG  VAL_H VAL_L   CHK
--    Leer     : A6  REG  00    00      CHK
--  FPGA -> Arduino
--    A medicion        : 55 MV STATUS CHK
--    A escribir / leer : 5A REG VAL_H VAL_L CHK   (REG con bit 7 = error)
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.lev_pkg.all;

entity arduino_link is
    generic (
        CLK_FREQ_HZ : integer := 50_000_000;
        BAUD_RATE   : integer := 9600
    );
    port (
        clk        : in  std_logic;
        rst_n      : in  std_logic;
        rxd        : in  std_logic;
        txd        : out std_logic;
        -- Hacia el PID
        meas_valid : out std_logic;
        meas_pv    : out std_logic_vector(15 downto 0);
        -- Desde el PID
        pid_done   : in  std_logic;
        mv         : in  std_logic_vector(7 downto 0);
        status     : in  std_logic_vector(7 downto 0);
        -- Banco de registros
        regs       : in  reg_array_t;
        wr_en      : out std_logic;
        wr_addr    : out integer range 0 to NUM_REGS - 1;
        wr_data    : out std_logic_vector(15 downto 0);
        -- Diagnostico
        frame_bad  : out std_logic   -- pulso: trama con error
    );
end entity arduino_link;

architecture rtl of arduino_link is
    constant CLKS_PER_BIT : integer := CLK_FREQ_HZ / BAUD_RATE;
    constant GAP_CLKS     : integer := CLKS_PER_BIT * 30;   -- ~3 ms entre bytes = trama rota

    type byte_arr is array (0 to 4) of std_logic_vector(7 downto 0);

    signal rx_data  : std_logic_vector(7 downto 0);
    signal rx_valid : std_logic;
    signal rx_ferr  : std_logic;

    signal rbuf     : byte_arr := (others => (others => '0'));
    signal rcnt     : integer range 0 to 5 := 0;
    signal rneed    : integer range 0 to 5 := 0;
    signal rxor     : std_logic_vector(7 downto 0) := (others => '0');
    signal gap_cnt  : integer range 0 to GAP_CLKS := 0;

    signal wait_pid : std_logic := '0';

    signal tbuf     : byte_arr := (others => (others => '0'));
    signal tlen     : integer range 0 to 5 := 0;
    signal tidx     : integer range 0 to 5 := 0;
    type tstate_t is (T_IDLE, T_LOAD, T_WAITB, T_WAITD);
    signal tstate   : tstate_t := T_IDLE;
    signal tx_start : std_logic := '0';
    signal tx_data  : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_busy  : std_logic;
begin

    u_rx : entity work.uart_rx
        generic map (CLKS_PER_BIT => CLKS_PER_BIT)
        port map (clk => clk, rst_n => rst_n, rxd => rxd,
                  data => rx_data, valid => rx_valid, frame_err => rx_ferr);

    u_tx : entity work.uart_tx
        generic map (CLKS_PER_BIT => CLKS_PER_BIT)
        port map (clk => clk, rst_n => rst_n, start => tx_start, data => tx_data,
                  txd => txd, busy => tx_busy);

    process (clk, rst_n)
        variable b      : std_logic_vector(7 downto 0);
        variable x      : std_logic_vector(7 downto 0);
        variable reg_n  : integer range 0 to 255;
        variable send   : boolean;
        variable t      : byte_arr;
    begin
        if rst_n = '0' then
            rcnt       <= 0;
            rneed      <= 0;
            gap_cnt    <= 0;
            wait_pid   <= '0';
            meas_valid <= '0';
            wr_en      <= '0';
            frame_bad  <= '0';
            tstate     <= T_IDLE;
            tx_start   <= '0';
            tlen       <= 0;
            tidx       <= 0;
            meas_pv    <= (others => '0');
            wr_addr    <= 0;
            wr_data    <= (others => '0');
        elsif rising_edge(clk) then
            meas_valid <= '0';
            wr_en      <= '0';
            frame_bad  <= '0';
            tx_start   <= '0';
            send       := false;
            t          := (others => (others => '0'));

            -- ---------------- Analizador de tramas ----------------
            if rx_ferr = '1' then
                rcnt      <= 0;
                frame_bad <= '1';
            elsif rx_valid = '1' then
                b := rx_data;
                gap_cnt <= 0;
                if rcnt = 0 then
                    if b = x"AA" then
                        rbuf(0) <= b; rxor <= b; rcnt <= 1; rneed <= 4;
                    elsif b = x"A5" or b = x"A6" then
                        rbuf(0) <= b; rxor <= b; rcnt <= 1; rneed <= 5;
                    end if;
                else
                    rbuf(rcnt) <= b;
                    x := rxor xor b;
                    rxor <= x;
                    if rcnt + 1 = rneed then
                        rcnt <= 0;
                        if x /= x"00" then
                            frame_bad <= '1';                 -- checksum incorrecto
                        elsif rbuf(0) = x"AA" then
                            meas_pv    <= rbuf(1) & rbuf(2);
                            meas_valid <= '1';
                            wait_pid   <= '1';
                        else
                            reg_n := to_integer(unsigned(rbuf(1)));
                            if rbuf(0) = x"A5" then
                                if reg_n < NUM_REGS and reg_writable(reg_n) then
                                    wr_addr <= reg_n;
                                    wr_data <= rbuf(2) & rbuf(3);
                                    wr_en   <= '1';
                                    t := (x"5A", rbuf(1), rbuf(2), rbuf(3), x"00");
                                else
                                    t := (x"5A", rbuf(1) or x"80", x"00", x"00", x"00");
                                end if;
                            else  -- A6: leer
                                if reg_n < NUM_REGS then
                                    t := (x"5A", rbuf(1), regs(reg_n)(15 downto 8), regs(reg_n)(7 downto 0), x"00");
                                else
                                    t := (x"5A", rbuf(1) or x"80", x"00", x"00", x"00");
                                end if;
                            end if;
                            t(4) := t(0) xor t(1) xor t(2) xor t(3);
                            send := true;
                            if tstate = T_IDLE then tlen <= 5; end if;
                        end if;
                    else
                        rcnt <= rcnt + 1;
                    end if;
                end if;
            elsif rcnt > 0 then
                -- Silencio demasiado largo en medio de una trama
                if gap_cnt = GAP_CLKS then
                    rcnt      <= 0;
                    frame_bad <= '1';
                else
                    gap_cnt <= gap_cnt + 1;
                end if;
            end if;

            -- ---------------- Respuesta a la medicion ----------------
            if pid_done = '1' and wait_pid = '1' then
                wait_pid <= '0';
                t := (x"55", mv, status, x"55" xor mv xor status, x"00");
                send := true;
                if tstate = T_IDLE then tlen <= 4; end if;
            end if;

            -- ---------------- Transmisor ----------------
            case tstate is
                when T_IDLE =>
                    if send then
                        tbuf   <= t;
                        tidx   <= 0;
                        tstate <= T_LOAD;
                    end if;
                when T_LOAD =>
                    if tidx < tlen then
                        tx_data  <= tbuf(tidx);
                        tx_start <= '1';
                        tstate   <= T_WAITB;
                    else
                        tstate <= T_IDLE;
                    end if;
                when T_WAITB =>
                    if tx_busy = '1' then tstate <= T_WAITD; end if;
                when T_WAITD =>
                    if tx_busy = '0' then
                        tidx   <= tidx + 1;
                        tstate <= T_LOAD;
                    end if;
            end case;
        end if;
    end process;

end architecture rtl;
